#include "swift/extractor/remapping/SwiftFileInterception.h"

#include <fcntl.h>
#include <filesystem>
#include <atomic>

#include <dlfcn.h>
#include <gnu/lib-names.h>
#include <mutex>

#include "swift/extractor/infra/file/FileHash.h"
#include "swift/extractor/infra/file/PathHash.h"

namespace fs = std::filesystem;

namespace codeql {

namespace {
template <typename Signature>
Signature getOriginalSyscall(const char* name) {
  auto libc = dlopen(LIBC_SO, RTLD_LAZY);
  return reinterpret_cast<Signature>(dlsym(libc, name));
}

auto& openInterceptor() {
  static std::weak_ptr<class FileInterceptor> ret{};
  return ret;
}

using OpenSignature = int (*)(const char*, int, ...);
using RenameSignature = int (*)(const char*, const char*);

int openOriginal(const char* path, int flags, mode_t mode = 0) {
  const static auto original = getOriginalSyscall<OpenSignature>("open");
  return original(path, flags, mode);
}

int renameOriginal(const char* oldpath, const char* newpath) {
  const static auto original = getOriginalSyscall<RenameSignature>("rename");
  return original(oldpath, newpath);
}

bool endsWith(const std::string_view& lhs, const std::string_view& rhs) {
  return lhs.size() >= rhs.size() && lhs.substr(lhs.size() - rhs.size()) == rhs;
}
}  // namespace

class FileInterceptor {
 public:
  FileInterceptor(fs::path&& workingDir) : workingDir{std::move(workingDir)} {
    fs::create_directories(hashes());
    fs::create_directories(store());
  }

  static int open(const char* path, int flags, int mode) {
    if (auto interceptor = openInterceptor().lock()) {
      return interceptor->openRedirected(path, flags, mode);
    } else {
      return openOriginal(path, flags, mode);
    }
  }

  static int rename(const char* source, const char* destination) {
    if (auto interceptor = openInterceptor().lock()) {
      return interceptor->renameRedirected(source, destination);
    } else {
      return renameOriginal(source, destination);
    }
  }

 private:
  int openRedirected(const char* path, int flags, mode_t mode = 0) const {
    auto accessMode = flags & O_ACCMODE;
    if (accessMode == O_RDONLY && endsWith(path, ".swiftmodule")) {
      if (auto fileHash = hash(path); !fileHash.empty()) {
        auto hashed = hashes() / fileHash;
        if (auto ret = openOriginal(hashed.c_str(), flags); ret >= 0 || errno != ENOENT) {
          return ret;
        }
      }
    }
    return openOriginal(path, flags, mode);
  }

  int renameRedirected(const char* source, const char* destination) {
    if (endsWith(destination, ".swiftmodule")) {
      if (auto fileHash = hash(destination); !fileHash.empty()) {
        auto hashed = hashes() / fileHash;
        auto target = store() / fs::path(destination).relative_path();
        fs::create_directories(target.parent_path());
        std::error_code ec;
        fs::create_symlink(target, hashed, ec);
        if (ec) {
          std::cerr << "Cannot remap file " << target << " -> " << hashed << ": " << ec.message()
                    << "\n";
        }
        return renameOriginal(source, target.c_str());
      }
    }
    return renameOriginal(source, destination);
  }

  fs::path hashes() const { return workingDir / "hashes"; }

  fs::path store() const { return workingDir / "store"; }

  std::string hash(const fs::path& target) const {
    {
      std::scoped_lock lock{cacheMutex};
      if (auto found = hashCache.find(target); found != hashCache.end()) {
        return found->second;
      }
    }
    std::string ret;
    if (auto fd = openOriginal(target); fd >= 0) {
      ret = hashFile(fd);
    }
    {
      std::scoped_lock lock{cacheMutex};
      hashCache.emplace(target, ret);
    }
    return ret;
  }

  fs::path workingDir;
  mutable std::mutex cacheMutex;
  mutable std::unordered_map<fs::path, std::string> hashCache;
};

int openOriginal(const std::filesystem::path& path) {
  return openOriginal(path.c_str(), O_RDONLY);
}

std::shared_ptr<FileInterceptor> setupOpenInterception(fs::path workingDir) {
  auto ret = std::make_shared<FileInterceptor>(std::move(workingDir));
  openInterceptor() = ret;
  return ret;
}
}  // namespace codeql

extern "C" {
int open(const char* path, int oflag, ...) {
  va_list ap;
  mode_t mode = 0;
  if ((oflag & O_CREAT) != 0) {
    // mode only applies to O_CREAT
    va_start(ap, oflag);
    mode = va_arg(ap, int);
    va_end(ap);
  }

  return codeql::FileInterceptor::open(path, oflag, mode);
}

int rename(const char* source, const char* destination) {
  return codeql::FileInterceptor::rename(source, destination);
}
}  // namespace codeql

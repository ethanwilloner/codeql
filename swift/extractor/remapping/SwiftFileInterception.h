#pragma once

#include <string>
#include <unordered_map>
#include <filesystem>

#include "swift/extractor/infra/file/PathHash.h"
#include <memory>

namespace codeql {

int openOriginal(const std::filesystem::path& path);

class FileInterceptor;

std::shared_ptr<FileInterceptor> setupOpenInterception(std::filesystem::path workingDir);

}  // namespace codeql

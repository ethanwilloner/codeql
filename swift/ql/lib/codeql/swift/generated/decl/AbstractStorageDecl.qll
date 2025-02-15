// generated by codegen/codegen.py
private import codeql.swift.generated.Synth
private import codeql.swift.generated.Raw
import codeql.swift.elements.decl.AccessorDecl
import codeql.swift.elements.decl.ValueDecl

module Generated {
  class AbstractStorageDecl extends Synth::TAbstractStorageDecl, ValueDecl {
    /**
     * Gets the `index`th accessor declaration of this abstract storage declaration (0-based).
     *
     * This includes nodes from the "hidden" AST. It can be overridden in subclasses to change the
     * behavior of both the `Immediate` and non-`Immediate` versions.
     */
    AccessorDecl getImmediateAccessorDecl(int index) {
      result =
        Synth::convertAccessorDeclFromRaw(Synth::convertAbstractStorageDeclToRaw(this)
              .(Raw::AbstractStorageDecl)
              .getAccessorDecl(index))
    }

    /**
     * Gets the `index`th accessor declaration of this abstract storage declaration (0-based).
     */
    final AccessorDecl getAccessorDecl(int index) {
      result = getImmediateAccessorDecl(index).resolve()
    }

    /**
     * Gets any of the accessor declarations of this abstract storage declaration.
     */
    final AccessorDecl getAnAccessorDecl() { result = getAccessorDecl(_) }

    /**
     * Gets the number of accessor declarations of this abstract storage declaration.
     */
    final int getNumberOfAccessorDecls() { result = count(getAnAccessorDecl()) }
  }
}

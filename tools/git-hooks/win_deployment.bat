@rem installation using symlinks
@if not exist ..\..\.git\hooks\ (
        mkdir ..\..\.git\hooks\
)
@if exist ..\..\.git\hooks\pre-commit (
        del ..\..\.git\hooks\pre-commit
)
@if exist ..\..\format-files-to-commit.rb (
        del ..\..\format-files-to-commit.rb
)

mklink ..\..\.git\hooks\pre-commit ..\..\tools\git-hooks\pre-commit-code.rb
mklink ..\..\format-files-to-commit.rb tools\git-hooks\format-files-to-commit.rb

@echo Git hook installation finished
@pause
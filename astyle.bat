for /R %%f in (*.c;*.h) do AStyle.exe --style=allman --indent=spaces=4 --pad-oper --pad-header --unpad-paren --suffix=none --align-pointer=name --lineend=windows --convert-tabs --verbose %%f
pause
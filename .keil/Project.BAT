SET PATH=C:\Keil_v5\ARM\ARMCLANG\Bin;C:\WINDOWS\system32;C:\WINDOWS;C:\WINDOWS\System32\Wbem;C:\WINDOWS\System32\WindowsPowerShell\v1.0\;C:\Program Files\dotnet\;C:\Program Files\Microsoft SQL Server\130\Tools\Binn\;C:\Program Files\Microsoft SQL Server\Client SDK\ODBC\170\Tools\Binn\;C:\Program Files\Git\cmd;C:\Program Files (x86)\NVIDIA Corporation\PhysX\Common;C:\Program Files\Calibre2\;C:\Program Files\LLVM\bin;C:\Users\ZGR\AppData\Local\Microsoft\WindowsApps;C:\Users\ZGR\AppData\Local\Programs\Microsoft VS Code\bin;
SET CPU_TYPE=NANO103SD3AE
SET CPU_VENDOR=Nuvoton
SET UV2_TARGET=Project
SET CPU_CLOCK=0x02255100
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\system_nano103.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\startup_nano103._ac"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\clk.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\fmc.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\gpio.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\spi.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\sys.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\timer.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\uart.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\adc.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\retarget.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\main.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\application.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmClang.exe" @".\objects\boot.__i"
"C:\Keil_v5\ARM\ARMCLANG\Bin\ArmLink" --Via ".\Objects\TableTennisX3_Receiver.lnp"
"C:\Keil_v5\ARM\ARMCLANG\Bin\fromelf.exe" ".\Objects\TableTennisX3_Receiver.axf" --i32combined --output ".\Objects\TableTennisX3_Receiver.hex"

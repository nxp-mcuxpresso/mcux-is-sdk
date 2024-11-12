'''
@author: B09332
'''

## Assumptions:
##   1.  Correct MCU SDK 2.0 branch has been check into the repositoryPath location below.
##   2.  The rby script to build the projects for the target has been executed (projects exist)

import os

targetPath = "C:\Coverity\issdk_frdmkl25z-onboard"

projectDirList = 	[
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmkl25z\\issdk_examples\\algorithms\\pedometer\\pedometer_mma8451\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmkl25z\\issdk_examples\\sensors\\mma8451q\\mma845x_fifo\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmkl25z\\issdk_examples\\sensors\\mma8451q\\mma845x_interrupt\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmkl25z\\issdk_examples\\sensors\\mma8451q\\mma845x_poll\\iar\\"
					];

projectIARList = 	[
                    "kl25z_mma8451_pedometer.ewp",
               		"mma845x_fifo_frdmkl25.ewp",
               		"mma845x_interrupt_frdmkl25.ewp",
               		"mma845x_poll_frdmkl25.ewp"
               		];

projectNames = 		[
                    "frdmkl25z-onboard_pedometer_mma8451",
                	"frdmkl25z-onboard_mma845x_fifo",
                	"frdmkl25z-onboard_mma845x_interrupt",
                	"frdmkl25z-onboard_mma845x_poll"
                	];

length = len(projectDirList)

for index in range(0,length):
    dir = projectDirList[index]
    os.chdir(projectDirList[index])
    print "Current Directory is " + os.getcwd()
    cmd = "cov-build --dir " + '"' + targetPath + "\\" + projectNames[index] + '"' + ' "' +"C:\\Program Files (x86)\\IAR Systems\\Embedded Workbench 7.5\\common\\bin\\IarBuild.exe" + '" ' +projectIARList[index] + " -build Debug"
    print "Coverity Build Command is: " + cmd
    os.system(cmd)
    cmd = "cov-analyze --dir " + '"' + targetPath + "\\" + projectNames[index] + '"' + " --all --enable-constraint-fpp --disable-parse-warnings -j auto"
    print "Coverity Analyze Command is: " + cmd
    os.system(cmd)
    cmd = "cov-commit-defects --dir "  + '"' + targetPath + "\\" + projectNames[index] + '"' + " --host coverity2.nxp.com --https-port 8443 --auth-key-file " + '"' +"C:\\Coverity\\auth-key\\cov.key" + '" ' + "--stream " + projectNames[index]
    print "Coverity Commit Command is: " + cmd
    os.system(cmd)

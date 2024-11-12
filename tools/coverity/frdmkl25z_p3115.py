'''
@author: B09332
'''

## Assumptions:
##   1.  Correct MCU SDK 2.0 branch has been check into the repositoryPath location below.
##   2.  The rby script to build the projects for the target has been executed (projects exist)

import os

targetPath = "C:\Coverity\issdk_frdmkl25-p3115"

projectDirList = 	[
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmkl25z_p3115\\issdk_examples\\sensors\\mpl3115\\mpl3115_altitude\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmkl25z_p3115\\issdk_examples\\sensors\\mpl3115\\mpl3115_fifo\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmkl25z_p3115\\issdk_examples\\sensors\\mpl3115\\mpl3115_fifo_interrupt\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmkl25z_p3115\\issdk_examples\\sensors\\mpl3115\\mpl3115_normal\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmkl25z_p3115\\issdk_examples\\sensors\\mpl3115\\mpl3115_normal_interrupt\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmkl25z_p3115\\issdk_examples\\sensors\\mpl3115\\mpl3115_oneshot\\iar\\",
					];

projectIARList = 	[               
					"mpl3115_altitude_frdmkl25-p3115.ewp",
					"mpl3115_fifo_frdmkl25-p3115.ewp",
					"mpl3115_fifo_interrupt_frdmkl25-p3115.ewp",
					"mpl3115_normal_frdmkl25-p3115.ewp",
					"mpl3115_normal_interrupt_frdmkl25-p3115.ewp",
					"mpl3115_oneshot_frdmkl25-p3115.ewp",
               		];
               
projectNames = 		[
					"frdmkl25-p3115_mpl3115_altitude",
					"frdmkl25-p3115_mpl3115_fifo",
					"frdmkl25-p3115_mpl3115_fifo_interrupt",
					"frdmkl25-p3115_mpl3115_normal",
					"frdmkl25-p3115_mpl3115_normal_interrupt",
					"frdmkl25-p3115_mpl3115_oneshot",
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
    
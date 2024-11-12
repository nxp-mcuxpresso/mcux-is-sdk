'''
@author: B09332
'''

## Assumptions:
##   1.  Correct MCU SDK 2.0 branch has been check into the repositoryPath location below.
##   2.  The rby script to build the projects for the target has been executed (projects exist)

import os

targetPath = "C:\Coverity\issdk_frdm-k64f-agm04"

projectDirList = [
			   "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_agm04\\issdk_examples\\algorithms\\pedometer\\pedometer_agm04\\iar\\",
			   "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_agm04\\issdk_examples\\algorithms\\sensorfusion\\freertos_agm04\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_agm04\\issdk_examples\\sensors\\fxas21002\\fxas21002_fifo\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_agm04\\issdk_examples\\sensors\\fxas21002\\fxas21002_interrupt\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_agm04\\issdk_examples\\sensors\\fxas21002\\fxas21002_poll\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_agm04\\issdk_examples\\sensors\\mag3110\\mag3110_normal\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_agm04\\issdk_examples\\sensors\\mag3110\\mag3110_normal_interrupt\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_agm04\\issdk_examples\\sensors\\mag3110\\mag3110_oneshot\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_agm04\\issdk_examples\\sensors\\mma865x\\mma865x_fifo\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_agm04\\issdk_examples\\sensors\\mma865x\\mma865x_interrupt\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_agm04\\issdk_examples\\sensors\\mma865x\\mma865x_freefall\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_agm04\\issdk_examples\\sensors\\mma865x\\mma865x_poll\\iar\\"
               ];

projectIARList = [               
			   "k64f_agm04_pedometer.ewp",
			   "k64f_agm04_freertos.ewp",
               "k64f_agm04_fxas21002_fifo.ewp",
               "k64f_agm04_fxas21002_interrupt.ewp",
               "k64f_agm04_fxas21002_poll.ewp",
               "k64f_agm04_mag3110_normal.ewp",
               "k64f_agm04_mag3110_normal_interrupt.ewp",
               "k64f_agm04_mag3110_oneshot.ewp",
               "k64f_agm04_mma865x_fifo.ewp",
               "k64f_agm04_mma865x_interrupt.ewp",
               "k64f_agm04_mma865x_freefall.ewp",
               "k64f_agm04_mma865x_poll.ewp"
               ];
               
projectNames = [
				"frdm-k64f-agm04_pedometer",
				"frdm-k64f-agm04_sensorfusion",
                "frdm-k64f-agm04_fxas21002_fifo",
                "frdm-k64f-agm04_fxas21002_interrupt",
                "frdm-k64f-agm04_fxas21002_poll",
                "frdm-k64f-agm04_mag3110_normal.ewp",
                "frdm-k64f-agm04_mag3110_normal_interrupt.ewp",
                "frdm-k64f-agm04_mag3110_oneshot.ewp",
                "frdm-k64f-agm04_mma865x_fifo",
                "frdm-k64f-agm04_mma865x_interrupt",
                "frdm-k64f-agm04_mma865x_freefall",
                "frdm-k64f-agm04_mma865x_poll"
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
    
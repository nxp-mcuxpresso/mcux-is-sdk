'''
@author: B09332
'''

## Assumptions:
##   1.  Correct MCU SDK 2.0 branch has been check into the repositoryPath location below.
##   2.  The rby script to build the projects for the target has been executed (projects exist)

import os

targetPath = "C:\Coverity\issdk_frdm-k22f-agm01"

projectDirList = [
			   "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agm01\\issdk_examples\\algorithms\\pedometer\\pedometer_agm01\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agm01\\issdk_examples\\algorithms\\sensorfusion\\freertos_agm01\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agm01\\issdk_examples\\algorithms\\sensorfusion\\baremetal_agm01\\iar\\",
			   "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agm01\\issdk_examples\\sensors\\fxas21002\\fxas21002_fifo\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agm01\\issdk_examples\\sensors\\fxas21002\\fxas21002_interrupt\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agm01\\issdk_examples\\sensors\\fxas21002\\fxas21002_poll\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agm01\\issdk_examples\\sensors\\fxos8700\\fxos8700_fifo\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agm01\\issdk_examples\\sensors\\fxos8700\\fxos8700_interrupt\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agm01\\issdk_examples\\sensors\\fxos8700\\fxos8700_poll\\iar\\"
               ];

projectIARList = [
               "k22f_agm01_pedometer.ewp",
               "k22f_agm01_freertos.ewp",
               "k22f_agm01_baremetal.ewp",
               "k22f_agm01_fxas21002_fifo.ewp",
               "k22f_agm01_fxas21002_interrupt.ewp",
               "k22f_agm01_fxas21002_poll.ewp",
               "k22f_agm01_fxos8700_fifo.ewp",
               "k22f_agm01_fxos8700_interrupt.ewp",
               "k22f_agm01_fxos8700_poll.ewp"
               ];

projectNames = [
				"frdm-k22f-agm01_pedometer",
				"frdm-k22f-agm01_sensorfusion",
				"frdm-k22f-agm01_baremetal_sensorfusion",
                "frdm-k22f-agm01_fxas21002_fifo",
                "frdm-k22f-agm01_fxas21002_interrupt",
                "frdm-k22f-agm01_fxas21002_poll",
                "frdm-k22f-agm01_fxos8700_fifo",
                "frdm-k22f-agm01_fxos8700_interrupt",
                "frdm-k22f-agm01_fxos8700_poll"
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

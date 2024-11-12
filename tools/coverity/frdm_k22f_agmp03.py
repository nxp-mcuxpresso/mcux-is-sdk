'''
@author: B09332
'''

## Assumptions:
##   1.  Correct MCU SDK 2.0 branch has been check into the repositoryPath location below.
##   2.  The rby script to build the projects for the target has been executed (projects exist)

import os

targetPath = "C:\Coverity\issdk_frdm-k22f-agmp03"

projectDirList = [
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\algorithms\\sensorfusion\\freertos_agmp03\\iar\\",
			   "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\fxas21002\\fxas21002_fifo\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\fxas21002\\fxas21002_interrupt\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\fxas21002\\fxas21002_poll\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\fxas21002\\fxas21002_poll_spi\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\fxls8962\\fxls8962_normal\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\fxls8962\\fxls8962_interrupt\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\fxls8962\\fxls8962_normal_spi\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\mag3110\\mag3110_normal\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\mag3110\\mag3110_normal_interrupt\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\mag3110\\mag3110_oneshot\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\mpl3115\\mpl3115_altitude\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\mpl3115\\mpl3115_fifo\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\mpl3115\\mpl3115_fifo_interrupt\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\mpl3115\\mpl3115_normal\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\mpl3115\\mpl3115_normal_interrupt\\iar\\",
               "C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk22f_agmp03\\issdk_examples\\sensors\\mpl3115\\mpl3115_oneshot\\iar\\"
               ];

projectIARList = [
               "k22f_agmp03_freertos.ewp",
               "k22f_agmp03_fxas21002_fifo.ewp",
               "k22f_agmp03_fxas21002_interrupt.ewp",
               "k22f_agmp03_fxas21002_poll.ewp",
               "k22f_agmp03_fxas21002_poll_spi.ewp",
               "k22f_agmp03_fxls8962_normal.ewp",
               "k22f_agmp03_fxls8962_interrupt.ewp",
               "k22f_agmp03_fxls8962_normal_spi.ewp",
               "k22f_agmp03_mag3110_normal.ewp",
               "k22f_agmp03_mag3110_normal_interrupt.ewp",
               "k22f_agmp03_mag3110_oneshot.ewp",
               "k22f_agmp03_mpl3115_altitude.ewp",
               "k22f_agmp03_mpl3115_fifo.ewp",
               "k22f_agmp03_mpl3115_fifo_interrupt.ewp",
               "k22f_agmp03_mpl3115_normal.ewp",
               "k22f_agmp03_mpl3115_normal_interrupt.ewp",
               "k22f_agmp03_mpl3115_oneshot.ewp"
               ];

projectNames = [
               "frdm-k22f-agmp03_freertos_sensorfusion",
               "frdm-k22f-agmp03_fxas21002_fifo",
               "frdm-k22f-agmp03_fxas21002_interrupt",
               "frdm-k22f-agmp03_fxas21002_poll",
               "frdm-k22f-agmp03_fxas21002_poll_spi",
               "frdm-k22f-agmp03_fxls8962_normal",
               "frdm-k22f-agmp03_fxls8962_interrupt",
               "frdm-k22f-agmp03_fxls8962_normal_spi",
               "frdm-k22f-agmp03_mag3110_normal",
               "frdm-k22f-agmp03_mag3110_normal_interrupt",
               "frdm-k22f-agmp03_mag3110_oneshot",
               "frdm-k22f-agmp03_mpl3115_altitude",
               "frdm-k22f-agmp03_mpl3115_fifo",
               "frdm-k22f-agmp03_mpl3115_fifo_interrupt",
               "frdm-k22f-agmp03_mpl3115_normal",
               "frdm-k22f-agmp03_mpl3115_normal_interrupt",
               "frdm-k22f-agmp03_mpl3115_oneshot"
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

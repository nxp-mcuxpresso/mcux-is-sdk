'''
@author: B09332
'''

## Assumptions:
##   1.  Correct MCU SDK 2.0 branch has been check into the repositoryPath location below.
##   2.  The rby script to build the projects for the target has been executed (projects exist)

import os

targetPath = "C:\Coverity\issdk_frdm-k64f-mult2b"

projectDirList = 	[
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\algorithms\\pedometer\\pedometer_mult2b\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\algorithms\\sensorfusion\\baremetal_mult2b\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\algorithms\\sensorfusion\\freertos_mult2b\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\fxas21002\\fxas21002_fifo\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\fxas21002\\fxas21002_interrupt\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\fxas21002\\fxas21002_poll\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\fxls8471q\\fxls8471q_spi_fifo\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\fxls8471q\\fxls8471q_spi_interrupt\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\fxls8471q\\fxls8471q_spi_poll\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\fxos8700\\fxos8700_fifo\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\fxos8700\\fxos8700_interrupt\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\fxos8700\\fxos8700_poll\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\mag3110\\mag3110_normal\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\mag3110\\mag3110_normal_interrupt\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\mag3110\\mag3110_oneshot\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\mma865x\\mma865x_fifo\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\mma865x\\mma865x_freefall\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\mma865x\\mma865x_interrupt\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\mma865x\\mma865x_poll\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\mma9553\\mma9553_pedometer_i2c\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\mpl3115\\mpl3115_altitude\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\mpl3115\\mpl3115_fifo\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\mpl3115\\mpl3115_normal\\iar\\",
					"C:\\Coverity\\mcu-sdk-2.0\\boards\\frdmk64f_mult2b\\issdk_examples\\sensors\\mpl3115\\mpl3115_oneshot\\iar\\"
					];

projectIARList = 	[      
					"pedometer_mult2b.ewp",         
					"baremetal_mult2b.ewp",
					"freertos_mult2b.ewp",
              		"fxas21002_fifo_mult2b.ewp",
               		"fxas21002_interrupt_mult2b.ewp",
               		"fxas21002_poll_mult2b.ewp",
               		"fxls8471q_spi_fifo_mult2b.ewp",
               		"fxls8471q_spi_interrupt_mult2b.ewp",
               		"fxls8471q_spi_poll_mult2b.ewp",
               		"fxos8700_fifo_mult2b.ewp",
               		"fxos8700_interrupt_mult2b.ewp",
               		"fxos8700_poll_mult2b.ewp",
					"mag3110_normal_mult2b.ewp",
					"mag3110_normal_interrupt_mult2b.ewp",
					"mag3110_oneshot_mult2b.ewp",
					"mma865x_fifo_mult2b.ewp",
					"mma865x_freefall_mult2b.ewp",
					"mma865x_interrupt_mult2b.ewp",
					"mma865x_poll_mult2b.ewp",
					"mma9553_pedometer_i2c_mult2b.ewp",
					"mpl3115_altitude_mult2b.ewp",
					"mpl3115_fifo_mult2b.ewp",
					"mpl3115_normal_mult2b.ewp",
					"mpl3115_oneshot_mult2b.ewp"
               		];
               
projectNames = 		[
					"frdm-k64f-mult2b_pedometer",
					"frdm-k64f-mult2b_baremetal",
					"frdm-k64f-mult2b_freertos",
               		"frdm-k64f-mult2b_fxas21002_fifo",
                	"frdm-k64f-mult2b_fxas21002_interrupt",
                	"frdm-k64f-mult2b_fxas21002_poll",
                	"frdm-k64f-mult2b_fxls8471q_spi_fifo",
                	"frdm-k64f-mult2b_fxls8471q_spi_interrupt",
                	"frdm-k64f-mult2b_fxls8471q_spi_poll"
                	"frdm-k64f-mult2b_fxos8700_fifo",
                	"frdm-k64f-mult2b_fxos8700_interrupt",
                	"frdm-k64f-mult2b_fxos8700_poll",
					"frdm-k64f-mult2b_mag3110_normal",
					"frdm-k64f-mult2b_mag3110_normal_interrupt",
					"frdm-k64f-mult2b_mag3110_oneshot",
					"frdm-k64f-mult2b_mma865x_fifo",
					"frdm-k64f-mult2b_mma865x_freefall",
					"frdm-k64f-mult2b_mma865x_interrupt",
					"frdm-k64f-mult2b_mma865x_poll",
					"frdm-k64f-mult2b_mma9553_pedometer_i2c",
					"frdm-k64f-mult2b_mpl3115_altitude",
					"frdm-k64f-mult2b_mpl3115_fifo",
					"frdm-k64f-mult2b_mpl3115_normal",
					"frdm-k64f-mult2b_mpl3115_oneshot"
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
    
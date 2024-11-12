'''
@author    : B35385
@name      : buildIARprojects
@copyright : Copyright (c) 2016, NXP Semiconductors.

********************************************************************************
This is the module used to build IAR project binaries from KEx Archives.
********************************************************************************
'''
import os
import sys
import time
import shutil
import zipfile

from buildConfiguration import FRDM_SHIELD_PROJECTS_PATH as ShieldProjectsPathFRDM
from buildConfiguration import LPC_SHIELD_PROJECTS_PATH as ShieldProjectsPathLPC
from buildConfiguration import QN_SHIELD_PROJECTS_PATH as ShieldProjectsPathQN
from buildConfiguration import FRDM_ARCHIVE_PATTERN as ArchivePatternFRDM
from buildConfiguration import FRDM_PROJECTS_PATH as OnBoardProjectsPath
from buildConfiguration import LPC_ARCHIVE_PATTERN as ArchivePatternLPC
from buildConfiguration import QN_ARCHIVE_PATTERN as ArchivePatternQN
from buildConfiguration import KEx_ARCHIVE_PATH as ArchiveBasePath
from buildConfiguration import BINARY_FILE_PATH as BinaryBasePath
from buildConfiguration import ARCHIVE_FILE_EXTN as ArchiveExtn

ArchiveMode     = 'r'
ArchiveFiles    = []
ArchivesFound   = 0

ProjectFileExtn    = ".ewp"
ProjectsFound      = 0

IARProjectFiles  = []
IARBuilderExe    = "IarBuild.exe "
IARBuilderTarget = " Debug"

BinaryFileExtn1 = ".bin"
BinaryFileExtn2 = ".srec"

def FindArchives():
    global ArchivesFound
    print("Search for Archives in directory :-\n[%s]..." % ArchiveBasePath)
    for root, _, archiveFiles in os.walk(ArchiveBasePath):
        for archiveFile in archiveFiles:
            if (archiveFile.startswith(ArchivePatternFRDM) or archiveFile.startswith(ArchivePatternLPC)) and archiveFile.endswith(ArchiveExtn):
                ArchiveFiles.append(os.path.join(root, archiveFile))
                ArchivesFound += 1
    print("Found [%d] Archives...\n" % ArchivesFound)

def UnZipAndFindProjectFiles():
    index = 0
    global ProjectsFound
    shutil.rmtree(BinaryBasePath, ignore_errors=True)
    time.sleep(1)
    os.makedirs(BinaryBasePath)
    for ArchiveFileName in ArchiveFiles:
        index += 1
        print("Unzip KEx Archive [%d/%d] :-\n[%s]..." % (index, ArchivesFound, ArchiveFileName))
        archive_file = zipfile.ZipFile(ArchiveFileName, ArchiveMode)
        archive_output_dir = BinaryBasePath + "\\" + ArchiveFileName.replace(ArchiveBasePath, '').replace(ArchiveExtn, '')
        archive_file.extractall(path=archive_output_dir)
        print("Unzip Complete...")
        sensorProjectsPaths = []
        if len(ArchiveFileName.replace(ArchiveExtn, '').split(ArchivePatternFRDM)) > 1:
            sensorProjectsPaths.append(ShieldProjectsPathFRDM.replace("board_shield", ArchiveFileName.replace(ArchiveExtn, '').split(ArchivePatternFRDM)[1].lower().replace('-','_')))
            sensorProjectsPaths.append(OnBoardProjectsPath.replace("shield", ArchiveFileName.replace(ArchiveExtn, '').split(ArchivePatternFRDM)[1].split('-')[0].lower()))
        if len(ArchiveFileName.replace(ArchiveExtn, '').split(ArchivePatternLPC)) > 1:
            sensorProjectsPaths.append(ShieldProjectsPathLPC.replace("board_shield", ArchiveFileName.replace(ArchiveExtn, '').split(ArchivePatternLPC)[1].lower().replace('-','_')))
        if len(ArchiveFileName.replace(ArchiveExtn, '').split(ArchivePatternQN)) > 1:
            sensorProjectsPaths.append(ShieldProjectsPathQN.replace("shield", ArchiveFileName.replace(ArchiveExtn, '').split(ArchivePatternQN)[1].lower()))
        for sensorProjectsPath in sensorProjectsPaths:
            projects = 0
            for root, _, projectFiles in os.walk(archive_output_dir + sensorProjectsPath):
                for projectFile in projectFiles:
                    if projectFile.endswith(ProjectFileExtn):
                        IARProjectFiles.append(os.path.join(root, projectFile))
                        projects += 1
            if projects > 0:
                print("\nSearched for IAR Projects in directory :-\n[%s]..." % (archive_output_dir + sensorProjectsPath))
                print("Found [%d] IAR Projects...\n" % projects)
                ProjectsFound += projects

def BuildProjects():
    print("Building all [%d] IAR Projects..." % ProjectsFound)
    index = 0
    for ProjectFile in IARProjectFiles:
        index += 1
        status = os.system(IARBuilderExe + ProjectFile + IARBuilderTarget)
        if(status == 0):
            sys.stdout.write("Build SUCCESS (%d/%d): %s\n" % (index, ProjectsFound, ProjectFile))
            sys.stdout.flush()
        else:
            sys.stderr.write("Build FAILED  (%d/%d): %s\n" % (index, ProjectsFound, ProjectFile))
            sys.stderr.flush()

def CopyBinaries():
    print("Search and Copy built Binaries for [%d] Projects found..." % ProjectsFound)
    binaries = 0
    for binaryPath in IARProjectFiles:
        for root, _, binaryFiles in os.walk(os.path.dirname(binaryPath)):
            for binaryFile in binaryFiles:
                if binaryFile.endswith(BinaryFileExtn1) or binaryFile.endswith(BinaryFileExtn2):
                    binaries += 1
                    shutil.copy(os.path.join(root, binaryFile), BinaryBasePath)
    print("Found and Copied  [%d] Binaries..." % binaries)

def Main():
    os.system("cls")
    FindArchives()
    UnZipAndFindProjectFiles()
    BuildProjects()
    CopyBinaries()

if __name__ == '__main__':
    Main()

'''
@author    : B35385
@name      : outputFormat
@copyright : Copyright (c) 2016, NXP Semiconductors.

************************************************************
This is the Test Summary and output format and display file.
************************************************************
'''

import time
import inspect      # Needed to get line number
import unittest     # Needed for assert

UNITY_FWK_SUMMARY_LINE_END   = 2  # 2nd Last Line.
UNITY_FWK_TEST_PASS_STR      = "OK\r\n"
UNITY_FWK_TESTS_COUNT_POS    = 0
UNITY_FWK_FAILURES_COUNT_POS = 2

DRIVER_EXAMPLE_SUMMARY_LINE_END   = 1  # Last Line.
DRIVER_EXAMPLE_TEST_PASS_STR      = " Specified samples processed, press any key to continue..."

HOST_DEMO_BOARD_LINE    = 3  # Third Last Line.
HOST_DEMO_SHIELD_LINE   = 2  # Second Last Line.
HOST_DEMO_RESPONSE_LINE = 1  # Last Line.

class OutputFormat(unittest.TestCase):

    # Assert type constants
    ASSERTTYPE_ASSERT       = 1
    ASSERTTYPE_EQUAL        = 2
    ASSERTTYPE_NOTEQUAL     = 3
    ASSERTTYPE_ALMOSTEQUAL  = 4
    ASSERTTYPE_FALSE        = 5
    ASSERTTYPE_GREATER      = 6
    ASSERTTYPE_GREATEREQUAL = 7
    ASSERTTYPE_LESS         = 8
    ASSERTTYPE_LESSEQUAL    = 9

    # Pass/fail status constants
    STATUS_PASSED           = "Passed"
    STATUS_FAILED           = "*Failed*"

    # Run mode constants
    RUNMODE_RELEASE         = 1
    RUNMODE_DEBUG           = 2

    # See only failure mode, bit setting
    # Valid values are powers of 2's so we can OR bits together
    ASSERT_SETTING_SEEALL          = 0x01
    ASSERT_SETTING_SEEONLYFAILURE  = 0x02



    runMode = RUNMODE_RELEASE

    assertMode = ASSERT_SETTING_SEEONLYFAILURE

    testname_prv = ""
    testdesc_prv = ""
    testresult_prv = ""
    test_subTests_prv = 0
    test_subFailures_prv = 0
    testStartTime_prv = 0
    testEndTime_prv = 0
    testFailed_linenum_prv = 0
    testStopOnFailure = False

    suite_namedesc          = []  # [0]-suite name, [1]-suite desc
    suite_testnames         = []
    suite_testresult        = []
    suite_subTests          = []
    suite_subFailures       = []
    suite_testtime_sec      = []
    suite_testfailed_linenum = []
    suite_starttime         = time.asctime( time.localtime(time.time()) )
    suite_endtime           = time.asctime( time.localtime(time.time()) )

    completeSuite_suitenames = []
    completeSuite_totaltests = []
    completeSuite_totalpassed = []
    completeSuite_totalfailed = []
    completeSuite_totaltime_sec = []
    completeSuite_starttime   = []

    placeHolder                = 0   # dummy place holder attribute to take up space



    def __init__(self):
        '''
        Constructor
        '''
        self.completeSuite_starttime.append(time.asctime( time.localtime(time.time()) ))

    def lineno(self):
        """Returns the current line number in our program."""
        return inspect.currentframe().f_back.f_lineno


    def getTimeHrMinSec(self, in_sec, totaltime):
        totaltime[0] = int(in_sec / 60 / 60)
        totaltime[1] = int(in_sec / 60)
        totaltime[2] = int(in_sec - (totaltime[0] * 60 * 60) - (totaltime[1] * 60))
        return

    def setRunMode(self, new_runmode):
        if new_runmode == self.RUNMODE_RELEASE:
            self.runMode = self.RUNMODE_RELEASE
        else:
            self.runMode = self.RUNMODE_DEBUG

    def setAssertMode(self, new_assertmode):
        if new_assertmode == self.ASSERT_SETTING_SEEONLYFAILURE:
            self.assertMode = self.ASSERT_SETTING_SEEONLYFAILURE
        else:
            self.assertMode = self.ASSERT_SETTING_SEEALL

    def parseHostDemoResults(self, boardString, shieldString, test_output):
        # The Host Demo prints the second last line as Board Name.
        # The Host Demo prints the last line as Shield Name.
        lineWiseOutput = test_output.splitlines()
        boardLine  = lineWiseOutput[len(lineWiseOutput)- HOST_DEMO_BOARD_LINE]
        shieldLine = lineWiseOutput[len(lineWiseOutput)- HOST_DEMO_SHIELD_LINE]
        # The Host Demo response has the last field as Board Name and Shield Name.
        responseString = lineWiseOutput[len(lineWiseOutput)- HOST_DEMO_RESPONSE_LINE].decode().split(':')[-1]

        if (boardString.lower() in responseString.lower()) and (shieldString.lower() in responseString.lower()) and (boardString.lower() in boardLine.lower()) and (shieldString.lower() in shieldLine.lower()):
            self.test_subTests_prv = 1
            self.test_subFailures_prv = 0
            return True
        else:
            self.test_subTests_prv = 0
            self.test_subFailures_prv = 1
            return False

    def parseDriverExampleResults(self, test_output):
        # The Driver Examples print the last line as summary.
        # Format: X Tests Y Failures Z Ignored
        lineWiseOutput = test_output.splitlines()
        summaryLine = lineWiseOutput[len(lineWiseOutput)- DRIVER_EXAMPLE_SUMMARY_LINE_END]

        if(summaryLine.startswith(DRIVER_EXAMPLE_TEST_PASS_STR)):
            self.test_subTests_prv = 1
            self.test_subFailures_prv = 0
            return True
        else:
            self.test_subTests_prv = 0
            self.test_subFailures_prv = 1
            return False

    def parseSubtestResults(self, test_output):
        # The unity framework prints the 2nd last line as summary.
        # Format: X Tests Y Failures Z Ignored
        lineWiseOutput = test_output.splitlines()
        summaryLine = lineWiseOutput[len(lineWiseOutput)- UNITY_FWK_SUMMARY_LINE_END].split(' ')

        self.test_subTests_prv = int(summaryLine[UNITY_FWK_TESTS_COUNT_POS])
        self.test_subFailures_prv = int(summaryLine[UNITY_FWK_FAILURES_COUNT_POS])
        return

    def parseStreamtestResults(self, test_output, test_samples):
        # The streaming tests print the number of in sequence samples received as the first word of the last line.
        # Format : [DUT] X Incremental sequence Temperature Samples Received.
        lineWiseOutput = test_output.splitlines()
        lastLine = lineWiseOutput[len(lineWiseOutput)-1].split(' ')

        if int(lastLine[1]) == test_samples:
            self.test_subTests_prv    = 1
            self.test_subFailures_prv = 0
            result = True
        else:
            self.test_subTests_prv    = 1
            self.test_subFailures_prv = 1
            result = False

        return result

    def suiteInit(self, suiteName, suiteDesc):

        self.suite_namedesc.insert(0,suiteName)
        self.suite_namedesc.insert(1,suiteDesc)
        #self.suite_namedesc.append(suiteDesc)
        self.completeSuite_suitenames.append(suiteName)

        # New suite, remove all current test cases.
        maxtests = len(self.suite_testnames)
        for _ in range (0, maxtests):
            self.suite_testnames.pop()
            self.suite_testresult.pop()
            self.suite_subTests.pop()
            self.suite_subFailures.pop()
            self.suite_testtime_sec.pop()
            self.suite_testfailed_linenum.pop()

        print "\n\n"
        print "*****************************************************************************"
        print "*****************************************************************************"
        print "                         SUITE START: ", self.suite_namedesc[0]
        print "*****************************************************************************"
        print "*****************************************************************************"
        print "\n\n"

        return


    def suiteEnd(self):

        time.sleep(1)
        totaltest = len(self.suite_testnames)
        totaltime_sec = 0
        totaltestpassed = 0
        totaltestfailed = 0

        self.suite_endtime           = time.asctime( time.localtime(time.time()) )

        for i in range (0, len(self.suite_testtime_sec)):
            totaltime_sec      += self.suite_testtime_sec[i]

            if self.suite_testresult[i] == self.STATUS_PASSED:
                totaltestpassed    += 1
            else:
                totaltestfailed    += 1

        # Get time in hr, min, sec
        totaltime = [0,0,0] # hr, min, sec
        self.getTimeHrMinSec(totaltime_sec, totaltime)

        print "\n\n"
        print "*****************************************************************************"
        print "                SUITE SUMMARY: ", self.suite_namedesc[0]
        print "*****************************************************************************"
        print "Suite             : ", self.suite_namedesc[0]
        print "Desc              : ", self.suite_namedesc[1]
        print ""
        print "Total tests       : %d" % totaltest
        print "Total test passed : %d" % totaltestpassed
        print "Total test failed : %d" % totaltestfailed
        print "Total test time   : %d hr" % totaltime[0], " %d min" % totaltime[1], " %d sec" % totaltime[2]
        print "Start time        : %s" % self.suite_starttime
        print "End time          : %s" % self.suite_endtime
        print ""

        print "------------------------------------------------------------------------------------"
        print "      Test                                               SubTest   Error            "
        print " #    Name                          Result     SubTests  Failures  Line    Time(sec)"
        print "------------------------------------------------------------------------------------"
        for i in range (0, len(self.suite_testnames)):
            if self.suite_testresult[i] == self.STATUS_PASSED:
                print "% -3i " % (i+1), "%-30s" % self.suite_testnames[i], "%-10s" % self.suite_testresult[i], "%-9i" % self.suite_subTests[i], "%-9i" % self.suite_subFailures[i], "-      ", "%-3.2f" % self.suite_testtime_sec[i]
            else:
                print "% -3i " % (i+1), "%-30s" % self.suite_testnames[i], "%-10s" % self.suite_testresult[i], "%-9i" % self.suite_subTests[i], "%-9i" % self.suite_subFailures[i], "%-7i" % self.suite_testfailed_linenum[i], "%-3.2f" % self.suite_testtime_sec[i]


        print ""
        print "************************************************************************************"
        print "************************************************************************************"
        print "\n\n"

        self.completeSuite_totaltests.append(totaltest)
        self.completeSuite_totalpassed.append(totaltestpassed)
        self.completeSuite_totalfailed.append(totaltestfailed)
        self.completeSuite_totaltime_sec.append(totaltime_sec)

        return


    def testInit(self, testname, testdesc, stopOnFailure=True):

        self.testname_prv = testname
        self.testdesc_prv = testdesc
        self.testresult_prv = self.STATUS_PASSED
        self.test_subTests_prv = 0
        self.test_subFailures_prv = 0
        self.testFailed_linenum_prv = 0
        self.testStopOnFailure = stopOnFailure
        self.assertMode = self.ASSERT_SETTING_SEEONLYFAILURE

        self.testStartTime_prv = time.clock()        # Record start time

        testStartTime = time.asctime( time.localtime(time.time()) )

        # Add test name to list.
        self.suite_testnames.append(testname)

        # For cases where test case is run individually, don't add suite name.
        suitename = ""
        if len(self.suite_namedesc) >= 2:
            suitename = self.suite_namedesc[0]

        print "\n"
        print "*******************************TEST START************************************"
        print "Test            : ", testname
        print "Desc            : ", testdesc
        print "Suite           : ", suitename
        print "Start time      : ", testStartTime
        print "Stop on failure : ", stopOnFailure
        print "-----------------------------------------------------------------------------"
        print "Test output: "
        print ""

        return

    def testEnd(self):

        self.testEndTime_prv = time.clock()        # Record end time

        deltatime = self.testEndTime_prv - self.testStartTime_prv

        self.suite_testtime_sec.append(deltatime)
        self.suite_testfailed_linenum.append(self.testFailed_linenum_prv)
        self.suite_subTests.append(self.test_subTests_prv)
        self.suite_subFailures.append(self.test_subFailures_prv)

        print ""
        print "=================================================================================="
        if self.testresult_prv == self.STATUS_PASSED:
            print "Test: %s" % self.testname_prv, "   SubTests: %d" % self.test_subTests_prv, "   Result: %s" % self.testresult_prv, "   Time: %3.2fs" % deltatime
            self.suite_testresult.append(self.STATUS_PASSED)
        else:
            print "Test: %s" % self.testname_prv, "   SubFailures: %d" % self.test_subFailures_prv, "   Result: %s" % self.testresult_prv, "   Line: %-4d" % self.testFailed_linenum_prv
            self.suite_testresult.append(self.STATUS_FAILED)

        print "*******************************TEST END*******************************************"
        print "\n\n"
        return

    def assert_(self, A, resultDesc, lineNo):
        self.assertCommon(self.ASSERTTYPE_ASSERT, A, self.placeHolder, resultDesc, self.placeHolder, lineNo)
        return

    def assertEqual_(self, A, B, resultDesc, lineNo):
        self.assertCommon(self.ASSERTTYPE_EQUAL, A, B, resultDesc, self.placeHolder, lineNo)
        return

    def assertNotEqual(self, A, B, resultDesc, lineNo):
        self.assertCommon(self.ASSERTTYPE_NOTEQUAL, A, B, resultDesc, self.placeHolder, lineNo)
        return

    def assertAlmostEqual(self, A, B, resultDesc, Delta, lineNo):
        self.assertCommon(self.ASSERTTYPE_ALMOSTEQUAL, A, B, resultDesc, Delta, lineNo)
        return

    def assertFalse(self, A, resultDesc, lineNo):
        self.assertCommon(self.ASSERTTYPE_FALSE, A, self.placeHolder, resultDesc, self.placeHolder, lineNo)
        return

    def assertGreater(self, A, B, resultDesc, lineNo):
        self.assertCommon(self.ASSERTTYPE_GREATER, A, B, resultDesc, self.placeHolder, lineNo)
        return

    def assertGreaterEqual(self, A, B, resultDesc, lineNo):
        self.assertCommon(self.ASSERTTYPE_GREATEREQUAL, A, B, resultDesc, self.placeHolder, lineNo)
        return

    def assertLess(self, A, B, resultDesc, lineNo):
        self.assertCommon(self.ASSERTTYPE_LESS, A, B, resultDesc, self.placeHolder, lineNo)
        return

    def assertLessEqual(self, A, B, resultDesc, lineNo):
        self.assertCommon(self.ASSERTTYPE_LESSEQUAL, A, B, resultDesc, self.placeHolder, lineNo)
        return


    def assertCommon(self, assertType, A, B, resultDesc, Deltain, lineNo):

        localresult = self.STATUS_PASSED
        localprintassert = True

        # Check results only if previous assert test passed or if user wants to continue if failure encountered
        if (self.testresult_prv == self.STATUS_PASSED) or (self.testStopOnFailure == False):
            try:
                if assertType == self.ASSERTTYPE_ASSERT:
                    assert(A)
                elif assertType == self.ASSERTTYPE_EQUAL:
                    #NOTE: Calling assertEqual() will give an error relating to _type_equality_funcs attributes.
                    #self.assertEqual(A, B)
                    if A != B:
                        assert(False)   # Assert error
                elif assertType == self.ASSERTTYPE_NOTEQUAL:
                    self.assertNotEqual(A, B)
                elif assertType == self.ASSERTTYPE_ALMOSTEQUAL:
                    self.assertAlmostEqual(A, B, None, None, Deltain)
                elif assertType == self.ASSERTTYPE_FALSE:
                    self.assertFalse(A)
                elif assertType == self.ASSERTTYPE_GREATER:
                    self.assertGreater(A, B)
                elif assertType == self.ASSERTTYPE_GREATEREQUAL:
                    self.assertGreaterEqual(A, B)
                elif assertType == self.ASSERTTYPE_LESS:
                    self.assertLess(A, B)
                elif assertType == self.ASSERTTYPE_LESSEQUAL:
                    self.assertLessEqual(A, B)

            except AssertionError:
                localresult = self.STATUS_FAILED
                self.testresult_prv = self.STATUS_FAILED        # Marked as failed
                # If first failure occurred already, then don't overwrite linenum.
                if self.testFailed_linenum_prv == 0:
                    self.testFailed_linenum_prv = lineNo
                if (self.runMode == self.RUNMODE_DEBUG):
                    assert(False)
                pass                                            # Do nothing, keep going.

            if (self.assertMode & self.ASSERT_SETTING_SEEONLYFAILURE):
                if (localresult == self.STATUS_PASSED):
                    localprintassert = False

            if (localprintassert == True):
                print 'Assert line: %-6d' % lineNo, "Result: %-10s" % localresult, "Desc:", resultDesc

        return


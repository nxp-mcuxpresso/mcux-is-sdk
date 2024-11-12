#!/usr/bin/env ruby
# 
# Copyright 2015,2016, Freescale Semiconductor, Inc.
# Copyright 2017 NXP.
#
# Redistribution and use in source and binary forms, with or without modification,
# are permitted provided that the following conditions are met:
# 
# o Redistributions of source code must retain the above copyright notice, this list
#   of conditions and the following disclaimer.
# 
# o Redistributions in binary form must reproduce the above copyright notice, this
#   list of conditions and the following disclaimer in the documentation and/or
#   other materials provided with the distribution.
# 
# o Neither the name of the copyright holder nor the names of its
#   contributors may be used to endorse or promote products derived from this
#   software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
# ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
# 
require 'fileutils'
require 'benchmark'
require 'thread'

def isSymlink?(file_path)
   ret = ""
   if (RUBY_PLATFORM =~ /mswin|mingw|cygwin/) # windows

      _path_tmp = file_path.split("/")
      _path = _path_tmp[0...-1].join("/")
      _file = _path_tmp[-1]

      if (_path.strip != "")
          ret = `cd /d #{_path} && dir /al #{_file}`
      else
          ret = `dir /al #{_file}`
      end

      if (ret["File Not Found"])
          return false
      else
          retarr_tmp = ret.split("[")
          retarr = retarr_tmp[-1].split("]")

          return retarr[0].gsub(/\\/,"/")
      end
   elsif (RUBY_PLATFORM =~ /linux|unix/) # linux
      if (File.symlink?(file_path))
          return File.readlink(file_path)
      else
          return false
      end
   elsif (RUBY_PLATFORM =~ /darwin|mac os/) # mac
      if (File.symlink?(file_path))
          return File.readlink(file_path)
      else
          return false
      end
   end

end

Applicable_formats_clang = [".c", ".h", ".cpp", ".hpp"]
Applicable_formats_eol   = (Applicable_formats_clang | [".yml"])
Clang_version = "3.7.0"
Clang_format = "clang-format"
#you can change path to clang-format (in case you don't have clang-format in your PATH variable), see bellow:
#Clang_format = "\"c:\\Program Files\\LLVM\\bin\\clang-format.exe\""
Ignore_list_path = ".clang-ignore"
$ignore_list

folder = File.dirname(__FILE__)
if (isSymlink?(__FILE__))
    folder = File.dirname(isSymlink?(__FILE__))
    folder_t = File.join(File.dirname(__FILE__),folder)
end
Eol_format = "ruby #{folder_t}/base-format.rb"

# debug & development options
Verbose = false
Debug = false


def read_configuration

    # Current position
    cwd=Dir.pwd
    puts "cwd: #{cwd}" if (Verbose)

    # Pre-commit script position position
    scrd=File.dirname(__FILE__)
    puts "file dir name: #{scrd}" if (Verbose)

    puts "Checking ignore file: #{cwd}/#{Ignore_list_path}" if (Verbose)
    if File.exist?("#{cwd}/#{Ignore_list_path}") then
        $ignore_list = File.read(Ignore_list_path)
        puts "Found ignorelist at #{cwd}/#{Ignore_list_path}" if (Verbose)
        puts "Content: #{$ignore_list}" if (Verbose)
        return 0
    end

    print "Cannot find file '.clang-ignore'\n"
    # continue even if .clang-ignore file has not been found
    return 0
end

def check_configuration
    ### limit check to specific branches - deactivated now
    # command = "git rev-parse --abbrev-ref HEAD"
    # git_branch_name = `#{command}`
    # unless git_branch_name.include? Enabled_branch then
    #        # Clang check is not enforcered on this branch
    #        return 0
    # end

    ### clang-format version
    command = Clang_format + " --version 2>&1"
    puts "Executing command: " + command if (Verbose)
    local_clang_version = `#{command}`
    if (
        local_clang_version["is not recognized"] || # Windows
        local_clang_version["not found"] # Linux (: not found) and mac (: command not found)
        )

        print "\n\n"
        print ":::::::::::::::::::::::::::::::::::::::\n"
        print "::                                   ::\n"
        print "::     C O M M I T   D E N I E D     ::\n"
        print "::                                   ::\n"
        print "::          N O   C L A N G          ::\n"
        print "::                                   ::\n"
        print "::         I N S T A L L E D         ::\n"
        print "::                                   ::\n"
        print ":::::::::::::::::::::::::::::::::::::::\n"
        print "\n\n"
        print "Please follow steps on wiki page and install clang first\n"
        print "\nWiki link: http://wiki.freescale.net/display/mcusw/Clang+pre-commit+hook\n"
        print "\nOr you can bypass this pre-commit check by using 'git commit --no-verify' command."
        return 1
    end


    git_mail = `git config --global user.email`
    git_name = `git config --global user.name`
    git_autocrlf = `git config --global core.autocrlf`
    git_mail.strip!
    git_name.strip!
    git_autocrlf.strip!
    git_autocrlf.downcase! if git_autocrlf!=""
    if (
        git_mail == "" ||
        git_name == "" ||
        git_autocrlf != "true"
        )

        print "\n\n"
        print ":::::::::::::::::::::::::::::::::::::::\n"
        print "::                                   ::\n"
        print "::     C O M M I T   D E N I E D     ::\n"
        print "::                                   ::\n"
        print "::    B A D   G I T   C O N F I G    ::\n"
        print "::                                   ::\n"
        print ":::::::::::::::::::::::::::::::::::::::\n"
        print "\n\n"

        if (git_mail == "")
            print "Your name is not defined in git config\nPlease, use following example command and set up your name:\n"
            print "\tgit config --global user.name \"John Doe\"\n\n"
        end
        if (git_name == "")
            print "Your e-mail is not defined in git config\nPlease, use following example command and set up your e-mail:\n"
            print "\tgit config --global user.email \"john.doe@nxp.com\"\n\n"
        end
        if (git_autocrlf != "true")
            print "Your autocrlf feature is disabled in git config\nPlease, use following example command and set up autocrlf properly:\n"
            print "\tgit config --global core.autocrlf true\n\n"
        end

        print "\nYou can bypass this pre-commit check by using 'git commit --no-verify' command.\n"

        return 1
    end



    git_detached_head = `git branch --contains HEAD`
    if (
        git_detached_head["detached"]
        )

        print "\n\n"
        print ":::::::::::::::::::::::::::::::::::::::\n"
        print "::                                   ::\n"
        print "::     C O M M I T   D E N I E D     ::\n"
        print "::                                   ::\n"
        print "::     D E T A C H E D   H E A D     ::\n"
        print "::                                   ::\n"
        print ":::::::::::::::::::::::::::::::::::::::\n"
        print "\n\n"


        print "You are trying to create commit on detached HEAD\n"
        print "Please, switch to HEAD of branch first !\n"

        print "\nYou can bypass this pre-commit check by using 'git commit --no-verify' command.\n"

        return 1
    end

    unless local_clang_version.include? Clang_version then
        puts "Local clang version: " + local_clang_version if (Verbose)
        print "System configuration check failed:"
        print "\nYou have " + local_clang_version
        print "The only supported clang-format version is " + Clang_version + "."
        print "\nUse correct 'clang-format' version or you can either bypass whole coding standard check by using 'git commit --no-verify' command or add file to ignore list (.clang-ignore)."
        return 1
    end
    return 0
end

def check_file file_path
    # ignore deleted or moved files
    unless File.exist?(file_path)
        puts "File " + file_path + " ignored because file not found. (Rename operation performed?)" if (Verbose)
        return 0
    end
    # run clang-format, get result into string and compare with original file
    original_code =  `git show :#{file_path}`
    command = "git show :#{file_path} | #{Clang_format}"
    puts "Executing command: " + command if (Verbose)
    formated_code = `#{command}`
    puts "Processing clang-format result for " + file_path if (Verbose)
    unless formated_code.eql? original_code then
        print "File " + file_path + " is not formated according to the coding standard.\n"
        return 1
    end
    return 0
end

def check_file_eol file_path
    # ignore deleted or moved files
    unless File.exist?(file_path)
        puts "File " + file_path + " ignored because file not found. (Rename operation performed?)" if (Verbose)
        return 0
    end

    # lieendings, trailing whitespaces at the end of line, empty line at the EOF
    puts "check: EOLs " + file_path if (Verbose)
    original_code =  `git show :#{file_path}`
    command = "git show :#{file_path} | #{Eol_format} 2>&1"
    puts "Executing command: " + command if (Verbose)
    formated_code = `#{command}`
    unless formated_code.eql? original_code then
        print "File " + file_path + " is not formated according to the best practice (trailing whitespaces, new line at the EOF).\n"
        return 1
    end
    return 0
end

def ignore_file file_path
    puts "Ignorelist: #{$ignore_list.to_s}" if Verbose
    return false unless $ignore_list

    $ignore_list.split("\n").each do |act_item|
        act_item.strip!
        next unless act_item.length > 0
        if ( file_path =~ /#{act_item}/ )
            puts "Ignoring " + file_path
            return true
        end
    end
    return false
end

def check_files_to_commit
    return_code = 0
    files_modified = `git diff-index --cached --name-only HEAD`.split("\n")
    files_modified_q = Queue.new
    files_modified.each { |path| files_modified_q.push path }
    puts "files_modified: #{files_modified.to_s}" if (Verbose)

    if (files_modified.include?(".gitmodules"))
        file_content = File.read(".gitmodules")
        test_arr = file_content.split(/\n/)
        if (test_arr.any?{ |val| /(?=.*url ?= ?)(?!.*sw-stash).*/ =~ val })
            print "\nCommiting of .gitmodules file which includes non-stash URLs is not allowed\n"
            print "\nBut you can bypass this pre-commit check by using 'git commit --no-verify' command.\n"
            return 1
        end
    end

    mutex = Mutex.new
    threads = (0...3).map do
        Thread.new do
            begin
                while file_path = files_modified_q.pop(true)
                    next if file_path.size == 0
                    next unless (Applicable_formats_clang|Applicable_formats_eol).include? File.extname(file_path)
                    next if ignore_file file_path

                    if (Applicable_formats_clang.include?(File.extname(file_path)))
                      result = check_file file_path
                      if (result > 0)
                          mutex.synchronize do
                              return_code += result
                          end
                      end
                    end

                    if (Applicable_formats_eol.include?(File.extname(file_path)))
                      result = check_file_eol file_path
                      if (result > 0)
                          mutex.synchronize do
                              return_code += result
                          end
                      end
                    end

                end
            rescue ThreadError
            end 
        end
    end
    threads.map(&:join)
    
    if (return_code > 0) then
                print "\nUse 'tools/git-hookds/format-files-to-commit.rb' script to format all code in the stage area."
        print "\nOr you can bypass this pre-commit check by using 'git commit --no-verify' command."                
    end
    return return_code
end

return_code = 0
time = Benchmark.realtime {
    return_code = read_configuration
    if (return_code == 0)
        return_code = check_configuration
    end
    if (return_code == 0)
        return_code = check_files_to_commit
    end
}

print "\nElapsed time " + time.to_s + "s"

if !Debug then
    exit return_code
else
    print "\nDebug is turned on - commit denied. Original return_code " + return_code.to_s
    exit 1
end
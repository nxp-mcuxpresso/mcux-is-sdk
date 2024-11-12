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
# o Neither the name of the copyright holder, Inc. nor the names of its
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

STDOUT.sync = true
STDERR.sync = true

def isSymlink?(file_path)
   ret = ""
   if (RUBY_PLATFORM =~ /mswin|mingw|cygwin/) # windows

      _path_tmp = file_path.split("/")
      _path = _path_tmp[0...-1].join("/")
      _file = _path_tmp[-1]

      if (_path.strip != "")
          ret = `cd /d #{_path} 2>&1 && dir /al #{_file} 2>&1`
      else
          ret = `dir /al #{_file} 2>&1`
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
Clang_format = "clang-format"
Ignore_list_path = ".clang-ignore"
$ignore_list
Verbose = false
# Prevent of formatting files
Debug = false

file = __FILE__
folder = File.dirname(__FILE__)


# Detection of git hooks submodule to call eol base formatter
if (isSymlink?(__FILE__))
    folder = File.dirname(isSymlink?(__FILE__))
end

$Eol_format = "ruby #{folder}/base-format.rb"

print "$Eol_format: #{$Eol_format}\n" if Verbose

def ignore_file(file_path)
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

def format_files_to_commit
    ct_files_done = 0
    files_modified = `git diff-index --cached --name-only HEAD`.split("\n")
    files_modified.each do |path|
        next if path.size == 0
        next unless File.exist?(path)

        # check if actual file is on the list of ignored files
        next if ignore_file path

        if (Applicable_formats_clang.include?(File.extname(path)))
            # run clang-format, get result into string and compare with original file
            command_dry_run = Clang_format+" " + path + ""
            original_code = File.read(path)
            formated_code = `#{command_dry_run}`

            if (!formated_code.eql?(original_code))
                # update file by clang-format
                command_update = Clang_format+" -i " + path + ""

                `#{command_update}` if Debug == false
                print "\File has been formated by clang #{path}\n"
            else
                ct_files_done+=1
            end
        end

        if (Applicable_formats_eol.include?(File.extname(path)))

            # update file by base-format

            command_update = $Eol_format+" \"" + path + "\""

            `#{command_update}` if Debug == false
            if ($?.exitstatus!=0)
                print "\nFile has been formated by base-formater #{path}\n"
            else
                ct_files_done+=1
            end

        end

    end

    if (files_modified.length == 0)
        print "\nNothing to do, list of supported modified files is empty\n"
    elsif (files_modified.length*2 == ct_files_done)
        print "\nNothing to do, all files are already formatted\n"
    end
end

# Current position
cwd=Dir.pwd

puts "Checking ignore file: #{cwd}/#{Ignore_list_path}" if (Verbose)
if File.exist?("#{cwd}/#{Ignore_list_path}") then
    $ignore_list = File.read("#{cwd}/#{Ignore_list_path}")
    puts "Found ignorelist at #{cwd}/#{Ignore_list_path}" if (Verbose)
end

def endwait
    if (!ARGV.include?("nopause"))
        puts("complete, press enter to end")
        $stdin.gets
    end
end

format_files_to_commit
endwait()

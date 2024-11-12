#!/usr/bin/env ruby

def execute(command)
    puts("executing command - '#{command}'")
    system(command)
    if ($?.exitstatus != 0)
        if (!ARGV.include?("nopause"))
            # user case - wait if any error occured
            $stderr.puts("executing failed - '#{command}'")
            $stderr.puts("press enter to continue or kill script by CTRL+C, in case of questions please contact either Vladimir(B35211) or Marian(B37642)")
            $stdin.gets
        else
            # scripting case - report problem and die
            abort("executing failed - '#{command}'")
        end
    end
    puts
end

def endwait
    if (!ARGV.include?("nopause"))
        puts("complete, press enter to end")
        $stdin.gets
    end
end

execute "git submodule sync"
execute "git submodule foreach --recursive git submodule sync"
execute "git submodule update --init --recursive --force"




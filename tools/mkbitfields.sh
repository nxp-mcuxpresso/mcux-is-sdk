#!/bin/gawk -f 


BEGIN {
   shiftLookup["0"] = 4
   shiftLookup["1"] = 0
   shiftLookup["2"] = 1
   shiftLookup["3"] = 0
   shiftLookup["4"] = 2
   shiftLookup["6"] = 1
   shiftLookup["7"] = 0
   shiftLookup["8"] = 3
   shiftLookup["C"] = 2
   shiftLookup["E"] = 1
   shiftLookup["F"] = 0

   maxRL = 0
   maskIdx = 0
}

/#ifndef/ {
       part = substr($2,0,index($2,"_H_")-1) 
       pl = length(part)
       print
       next
}

/#endif/ {
       print
       next
}
      
#start of comment 
#
/^[ \t]*\/\*/ {
   comment=1
}

#inside a multi-line comment
#
comment==1 && /\*\// {
   comment=0;
   print
   next }


# a blank line - used to detect end of a register description
#
inType==1 && /^$/ { 

             printf("typedef uint8_t %s_%s_t;\n\n",part,reg)
             inType = 0
}

inType==2 && /^$/ { 
       bit = 0
       padCnt = 0;
       mwidth = 0;
       for (s in a)
       {
          if (bit != shft[s])
          { 
             if ((shft[s]-bit) < 0)
             {
                 printf("Error: negative bit field width (%d) in %s.%s\n",padCnt,reg,a[s]) >> "/dev/stderr"
             }
             else
             { 
                if ( padCnt ) {
                    padName = sprintf("_reserved_%1d",padCnt++);
                }
                else
                {
                    padName = sprintf("_reserved_");
                    padCnt++;
                }
                printf("        uint8_t %-21s :%2d;\n",padName,shft[s]-bit);     
                bit += shft[s]-bit
             }
          }
          if ((shft[s] == 0) && (width[s] == 8))
          {
             l = sprintf("        uint8_t  %20s;",tolower(a[s]));
          }
          else
          {
             l = sprintf("        uint8_t  %20s :%2d;",tolower(a[s]),width[s]);
          }
          c =formatComment(commentList[s],length(l),120)
          printf("%s%s\n",l,c)
          bit = shft[s]+width[s]
          if ( length(a[s]) > mwidth) mwidth = length(a[s])
       }
       print "    } b;"
       print "    uint8_t w;"
       printf("} %s_%s_t;\n\n",part,reg);
       inType=0

       print;
       printf("/*\n** %s - Bit field mask definitions \n*/\n",reg);
       for (s in a)
       {
           print createMaskShiftDefines(maskList[s],part, reg, a[s], mwidth)
       }
       delete maskList
       print
       if (bfIdx > 0)
       {
           printf("/*\n** %s - Bit field value definitions  \n*/\n",reg);
       }
       for (d in bfList)
       {
           printf("%s",bfList[d]);
       }
       delete bfList;
       bfIdx=0
       print "/*------------------------------*/"
       print; print;
       next
}

/enum/ { 
          regList=1
          print
          next
        }
regList==1 && /};/ {
          regList=0
          print
          next
        }
regList==1 {
          gsub("{","");
          f = substr($1, pl+2, length($1))
          if ( length($1) ) regNames[f] = 1
          offStart = index($0,"=")+1
          offStop = index($0,",")
          regOffset[f]=substr($0,offStart,offStop-offStart)
          print
          next 
        }

/#sa/ {
       os = sprintf("#define %s_DEVICE_ADDR_%-*s (0x%2s) ",part,20-length($2),toupper($2),$3);
       $1=""
       $2=""
       $3=""
       c = formatComment($0,length(os),120)
       printf("%s%s\n",os,c)
}

/#who/ {
       os = sprintf("#define %s_WHO_AM_I_%-*s (0x%2s) ",part,20-length($2),toupper($2),$3);
       $1=""
       $2=""
       $3=""
       c = formatComment($0,length(os),120)
       printf("%s%s\n",os,c)
}

# a C++ style comment - used to detect the start of a register description and get the reg name
#
comment==0 && /^\/\// { 
       if (inType==1)
       {
            printf("typedef uint8_t %s_%s_t;\n\n",part,reg);
            inType = 0;
       }
       reg = $2
       if ( length(reg) > maxRL) maxRL = length(reg)
       caseSuffix = index(reg, "__")
       if ( caseSuffix > 0 )
       {
            regBase = substr(reg,0,caseSuffix-1)
       }
       else
       {
           regBase = reg
       }
       regNames[regBase] = 0
       inType = 1
       printf("\n/*--------------------------------\n")
       printf("** Register: %s\n",reg)
       printf("** Enum: %s_%s\n",part,reg)
       printf("** --\n")
       $1 = "**"
       $2 = "Offset"
       if (match($0,"0x[0-9A-F]") > 0)
       {
           print $0
       } else
       {
           print $0, ":", regOffset[reg] 
       }
       printf("** ------------------------------*/\n" )
       delete a
       delete maskList
       next
} 
# a OR-able bit field value definition 
#
/#val/ { 
       bfName = toupper($2)
       bfValue = $3
       os = sprintf("#define %s_%s_%s_%-*s ((uint8_t) 0x%02x) ",part,toupper(reg),toupper(a[sh]),20-length(a[sh]),bfName,lshift(strtonum(bfValue),shft[sh]));
       bfList[bfIdx++] = os
       $1=""
       $2=""
       $3=""
       if (match($0,"[^ ]"))
       {
          bfList[bfIdx++] = formatComment($0,length(os),120)
       }
       else
       {
          bfList[bfIdx++] = "\n"
       }
       next
}

# a bit field mask definition - used to compute the width of the bit field
#
/#add/ { 
       if ( inType == 1)
       {
	  print "typedef union {"
	  print "    struct {"
          inType = 2
       }
       f = $2
       mask = $3
       w = getWidth(mask,reg,f);
       if ( w == 0 ) { printf(" register %s, field %s\n",reg,f) >> "/dev/stderr"; next }
       sh=getShiftSize(mask)
       shft[sh]=sh
       a[sh] =  f
       width[sh] = w
       maskList[sh] = mask
       $1=""; $2=""; $3=""
       commentList[sh] = substr($0,index($0,$4),length($0))  
       next
}
 
/#define.*_MASK/ { 
       if ( inType == 1)
       {
	  print "typedef union {"
	  print "    struct {"
          inType = 2
       }
       mask = $3
       gsub("[()]","",mask)
       f = substr($2, pl+2, length($2))
       gsub("_MASK","",f) 
       w = getWidth(mask,reg,f);
       if ( w == 0 ) { printf(" register %s, field %s\n",reg,f) >> "/dev/stderr"; next }
       sh=getShiftSize(mask)
       shft[sh]=sh
       a[sh] =  f
       width[sh] = w
       maskList[sh] = mask
       next
}
 
# ignore any existing bitfield shift definition
#
/#define.*_SHIFT/ {
       next
}

# Just echo any other line through to the output
#
{ print $0 }

END {
     if (length(regNames) > 0)
     {
        print "No fields found for:" >> "/dev/stderr"
		for ( r in regNames )
		{
		    if ( regNames[r] == 1 )
		    {
			   printf("%*s_%-*s : %s\n",9+pl,part,maxRL+1,r,regOffset[r])  >> "/dev/stderr"
			   printf("// %s : %s\n",r,regOffset[r])
		    }
		}
     }
}

function createMaskShiftDefines(mask, part, reg, f, w)
{
   maskName= toupper(f) "_MASK"
   shiftName= toupper(f) "_SHIFT"
   sv = getShiftSize(mask)
   m= sprintf("#define %s_%s_%-*s  ((uint8_t) %s)\n",part,reg,w+6,maskName,mask)
   s= sprintf("#define %s_%s_%-*s  ((uint8_t) %4s)\n",part,reg,w+6,shiftName,sv)
   return m s
}

function getShiftSize(mask, sh)
{
   sh=shiftLookup[substr(mask,length(mask),1)]
   if (sh == 4) sh+=shiftLookup[substr(mask,length(mask)-1,1)]
   return sh
}

function formatComment(c,osl,w,pad)
{
   sub(/ */,"",c)
   pad=1
   n=split(c,ca," ")
   maxcfw=w-osl
   cfw=0
   defStr=""
   for (i=1; i<=n;)
   {
       if (ca[i] == "\\n")
       {
           defStr = defStr sprintf("%*s */\n",maxcfw-cfw,"");
           cfw=0;
           pad = osl+1;
           i++
       }
       if (cfw==0)
       {
            defStr = defStr sprintf("%*s/* ",pad,"") 
            cfw +=3
       }
       if (cfw+length(ca[i]) < maxcfw)
       {
           defStr = defStr sprintf(" %s",ca[i]);
           cfw += length(ca[i])+1
           i++
       }
       else
       {
           defStr = defStr sprintf("%*s */\n",maxcfw-cfw,"");
           cfw=0;
           pad = osl+1;
       }
   }
   if (cfw) defStr = defStr sprintf("%*s */\n",maxcfw-cfw,"");

   return defStr
}

function getWidth(mask,reg,f,w )
{
       w = 0;
       for ( i=1; i<=length(mask); i++)
       {
          switch (substr(mask,i,1))
          {
              case "1":
              case "2":
              case "4":
              case "8":
                w += 1;
                break;
              case "3":
              case "6":
              case "C":
                w += 2;
                break;
              case "7":
              case "E":
                w += 3;
                break;
              case "F":
                w += 4;
                break;
              case "0":
              case "x":
                break;
              default:
                printf("Bad mask value (%s) in",substr(mask,i,1)) >> "/dev/stderr"
                printf(" register %s, field %s\n",reg,f) >> "/dev/stderr" 
                w = 0
           }
       }
    return w
}

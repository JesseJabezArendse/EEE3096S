import os

optimization_flags = [
    "-O0",
    "-O1",
    "-O2",
    "-O3",
    "-Ofast",
    "-Os",
    "-Og",
    "-funroll-loops"
]

commands = [
"make",
"make threaded",
"make run",
"make run_threaded"
]

currentfileString = "makefile original.mak"
currentfile = open(currentfileString)
currentLines = currentfile.readlines()



for flag in optimization_flags:
    outfile = open("makefile","w")
    
    print("********************************")
    print("current flag = ",flag)
    
    for line in currentLines:
  #      line = line.replace("    ","\t")
        line = line.replace("$(CFLAGS)","$(CFLAGS) "+flag)
        outfile.write(line)

    outfile.close()

    for cmd in commands:
        os.system(cmd)
        

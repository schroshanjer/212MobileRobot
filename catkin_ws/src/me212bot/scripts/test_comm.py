import subprocess
process = subprocess.Popen("/bin/bash", shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE);

process.stdin.write("ls\n")
process.stdin.write("cd ..\n")
process.stdin.flush()

#print process.stdout.flush()
# stdout, stderr = process.communicate()

# print "stdout: " + stdout
# print "stderr: " + stderr

process.stdin.write("ls\n")
process.stdin.flush()
stdout, stderr = process.communicate()

print bool(stderr)

print "stdout: " + stdout
print "stderr: " + stderr
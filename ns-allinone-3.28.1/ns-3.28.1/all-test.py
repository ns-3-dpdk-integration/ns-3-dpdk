import os
import sys
import shlex
from pwn import *

NUMTESTS = {
  "tcp": 15,
  # "udp": 1
}

TESTS = [   
  # {
  #   "name": "testdpdk",
  #   "dir": "/home/student/Desktop/harsh/Integration-of-DPDK-in-ns-3/ns-allinone-3.28.1/ns-3.28.1"
  # }
  {
    "name": "testfd",
    "dir": "/home/student/Desktop/harsh/Integration-of-DPDK-in-ns-3/ns-allinone-3.28.1/ns-3.28.1"
  }
  # {
  #   "name": "testnm",
  #   "dir": "/home/student/Desktop/harsh/netmap/ns-3-dev-git"
  # }
]

PROTS = [ 
  "tcp"
  # "udp"
]

SPEEDS = [ 
  "1000"
]

def get_remote_command(command):
    command = "ssh student@192.168.43.1 '{0}'".format(command)
    return shlex.split(command)

def get_command(command):
    return shlex.split(command)

def run_command(command, throw=True):
    err = process(get_command(command)).poll(block=True)
    if (err != 0) and throw:
        # sys.exit("Error in command\n\t{0}".format(command))
        process(get_command(command)).interactive()

def start_vtune(output, pid):
    print("Starting vtune-analyze")
    command = 'sh -c "~/bin/vtune-analyse ' + output + ' ' + pid + '"'
    return process(get_command(command))

def do_test(test, prot, speed, i):
    command = test["name"] + prot + speed
    print("Starting test #{0} for {1}".format(i+1, command))
    
    # Start test
    print("Starting test process")
    test_process = process(get_command("sh -c ~/bin/" + command))
    # test_process.interactive()
    test_process.recvuntil("(pid: ")
    pid = test_process.recvuntil(")")[:-1]
    print("Process pid {0}".format(pid))
    
    # Start vtune
    vtune_result = "{0}/{1}-vtune".format(test["dir"], command)
    vtune_process = start_vtune(vtune_result, pid)
    # vtune_process.interactive()
    vtune_process.recvuntil('Collection started.')

    # Resume test
    test_process.sendline("")

    # Wait for test to finish
    err = test_process.poll(block=True)
    if (err != 0):
        sys.exit("Error in test")

    # Wait for vtune
    # vtune_process.interactive()
    err = vtune_process.poll(block=True)
    if (err != 0):
        sys.exit("Error in vtune")

    # Save the results
    results_dir = "/home/student/Desktop/harsh/dpdk-results/{0}-{1}".format(command, i)
    run_command("mkdir -p {0}".format(results_dir))

    run_command("mv {0}/fd-client-0-0.pcap {1}/".format(test["dir"], results_dir))
    if prot == "tcp":
      run_command("mv {0}/cwnd.plotme {1}/".format(test["dir"], results_dir))
      run_command("mv {0}/sst.plotme {1}/".format(test["dir"], results_dir))
      run_command("mv {0}/inflight.plotme {1}/".format(test["dir"], results_dir))
    os.system("mv {0}/drops.plotme {1}/".format(test["dir"], results_dir))
    run_command("sudo mv {0} {1}/".format(vtune_result, results_dir))

    print("DONE\n\n\n")




for prot in PROTS:
    for test in TESTS:
        for speed in SPEEDS:
            for i in range(NUMTESTS[prot]):
                do_test(test, prot, speed, i)
            raw_input("Start next set of test???")

#! /usr/bin/python3

import threading
import queue
import time
import psutil
import subprocess
import sys

message_queue = queue.Queue()

def read_input():
    while True:
        try:
            raw_message = input()
        except EOFError:
            continue

        message = {
            "time": time.time(),
            "prefix": raw_message.split(":")[0],
            "tone": raw_message.split(":")[1].strip()
        }
        message_queue.put(message)
        print("Put message {}:{} on queue @ {}".format(message["prefix"], message["tone"], message["time"]))


# Stops Transmisson if Running
def handle_tone_BAD99():
    radioafsk_process.suspend()


# Starts Transmisson if Stopped
def handle_tone_MAD89():
    radioafsk_process.resume()

tone_functions = {
        "BAD99": handle_tone_BAD99,
        "DAB89": handle_tone_MAD89
}

radioafsk_process = None

if __name__ == "__main__":
    try:
        radioafsk_pid = int(subprocess.check_output(["pidof", "radioafsk"]))
    except subprocess.CalledProcessError:
        print("Please start radioafsk before this program!")
        sys.exit(1)

    radioafsk_process = psutil.Process(radioafsk_pid)


    input_thread = threading.Thread(target=read_input)
    input_thread.start()

    final_tone_length = 5
    max_message_delta = 1.2 # time in seconds
    last_message = {"prefix": "", "tone": "", "time": 0}
    tones = []

    while True:
        message = message_queue.get(True) # Get a message. Will block until one is available.

        # If this is the first message we've read
        # or
        # more than a fifth of a second has passed then
        # make the current message the last thing we've read
        # and make its tone the last tone that was recieved
        if last_message["time"] == 0 or message["time"] - last_message["time"] > max_message_delta:
            last_message = message
            tones = [last_message["tone"]]
            continue

        # If a message is recieved that has an incorrect
        # prefix, then reset tones and last_message to be
        # like we never read anything.
        if message["prefix"] != "DTMF":
            tones = []
            last_message = {"prefix": "", "tone": "", "time": 0}
            continue

        # Message should be good.
        # Put its tone onto the list of
        # recieved tones.
        tones.append(message["tone"])

        print("".join(tones))

        # If we've recieved a complete tone,
        # print it for now and reset the list
        # of recieved tones and the message
        # that has been read
        if len(tones) == final_tone_length:
            final_tone = "".join(tones)

            print(final_tone)
            if final_tone in tone_functions:
                tone_functions[final_tone]()

            tones = []
            last_message = {"prefix": "", "tone": "", "time": 0}
            continue

        last_message = message

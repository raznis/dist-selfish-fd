#!/usr/bin/python

import sys

expanded = 0
generated = 0
messages = 0
time = 0.0
plan_length = 0
init_h = 0
for file in range(int(sys.argv[1])):  
  f= open (str(file),"r")
  data = f.read()

  #Print it
  #print data
  #print data[data.find("Expanded ")+9 : data.find(" ", data.find("Expanded ")+9)]
  expanded = expanded + int(data[data.find("Expanded ")+9 : data.find(" ", data.find("Expanded ")+9)])
  generated = generated + int(data[data.find("Generated ")+10 : data.find(" ", data.find("Generated ")+10)])
  messages = messages + int(data[data.find("Messages received: ")+19 : data.find("\n", data.find("Messages received: ")+19)])
  time = max(time, float(data[data.find("Search time: ")+13 : data.find("s", data.find("Search time: ")+13)]))
  #init_h = max(init_h, int(data[data.find("Initial state h value: ")+23 : data.find(".", data.find("Initial state h value: ")+23)]))
  if data.find("Found solution with g=") > -1:
    plan_length = int(data[data.find("Found solution with g=")+22 : data.find(",", data.find("Found solution with g=")+22)])
  #print expanded, generated, messages, time
  # Close the file
  f.close() 
print "Expanded Generated messages time cost init_h"
print expanded, generated, messages, time, plan_length, init_h

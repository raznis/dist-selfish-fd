#!/usr/bin/python

import sys

expanded = 0
generated = 0
messages = 0
time = 0.0

for file in range(int(sys.argv[1])):  
  f= open (str(file),"r")
  data = f.read()

  #Print it
  #print data
  #print data[data.find("Expanded ")+9 : data.find(" ", data.find("Expanded ")+9)]
  expanded = expanded + int(data[data.find("Expanded ")+9 : data.find(" ", data.find("Expanded ")+9)])
  generated = generated + int(data[data.find("Generated ")+10 : data.find(" ", data.find("Generated ")+10)])
  messages = messages + int(data[data.find("Messages received: ")+19 : data.find("\n", data.find("Messages received: ")+19)])
  time = max(time, float(data[data.find("Total time: ")+12 : data.find("s", data.find("Total time: ")+12)]))
  #print expanded, generated, messages, time
  # Close the file
  f.close() 
print "Expanded Generated messages time"
print expanded, generated, messages, time
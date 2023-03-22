#!/usr/bin/env python3
import sys
sys.path.append('/root/zmqRemoteApi/clients/python')
from zmqRemoteApi import RemoteAPIClient

# create a client to connect to zmqRemoteApi server:
# (creation arguments can specify different host/port,
# defaults are host='localhost', port=23000)
client = RemoteAPIClient()

# get a remote object:
sim = client.getObject('sim')

# call API function:
h = sim.getObject('/fetch_respondable')
print(h)
''' This is an incredibly simple port forwarder from port 7000 to 22 on
localhost.  It calls a callback function when the socket is closed, to
demonstrate one way that you could start to do interesting things by
starting from a simple framework like this.
'''

import eventlet
import time


def closed_callback():
    print("called back")


def forward_c(source, dest, cb=lambda: None):
    """Forwards bytes unidirectionally from source to dest"""
    prv_d=''
    while True:
        d = source.recv(32384)
        if d == '':
            cb()
            break
        ln = len(d)
        if ln > 32000:
            continue
            #print( time.strftime('%X'),' c:',len(d))
            if len(prv_d) < 32000:
                print(prv_d)
            print(d[0:31])
        prv_d=d    
        dest.sendall(d)

def forward_s(source, dest, cb=lambda: None):
    """Forwards bytes unidirectionally from source to dest"""
    while True:
        d = source.recv(32384)
        if d == '':
            cb()
            break
        print( time.strftime('%X'),' s:',d)
        dest.sendall(d)

listener = eventlet.listen(('localhost', 4567))
while True:
    client, addr = listener.accept()
    server = eventlet.connect(('192.168.1.6', 4567))
    # two unidirectional forwarders make a bidirectional one
    eventlet.spawn_n(forward_c, client, server, closed_callback)
    eventlet.spawn_n(forward_s, server, client)
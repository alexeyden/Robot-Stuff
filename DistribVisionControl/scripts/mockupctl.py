#!/bin/env python3

import http.client
import sys

action = sys.argv[1]
target = sys.argv[2]

params = {"-action" : action, "-name" : target }

for name, val in zip(sys.argv[3::2], sys.argv[4::2]):
    params[name] = val

js = '{'
for p,v in params.items():
    try: vs = float(v)
    except ValueError:
        if not v.startswith('['):
            vs = '"' + v + '"'
        else:
            vs = v

    js += ('' if js == '{' else ',') + '"' + p[1:] + '":' + str(vs)
js += '}'

con = http.client.HTTPConnection('localhost:8080')
con.request('POST', '', js, {'Content-Type': 'application/json'})
resp  = con.getresponse()
print(resp.read().decode('utf-8'))

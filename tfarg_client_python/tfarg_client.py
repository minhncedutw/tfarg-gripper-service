import requests
import json
import numpy as np

URL = 'http://localhost:5001/v1/tfg/'
headers = {'content-type': 'application/json', 'Accept-Charset': 'UTF-8'} # config header of REST API


data = True
res = requests.put(url=URL + 'state/', data=json.dumps(data), headers=headers)
print(res.status_code, res.text)


data = 1
res = requests.put(url=URL + 'mode/', data=json.dumps(data), headers=headers)
print(res.status_code, res.text)


data = {
  "pos": 80,
  "speed": 110,
  "force": 20
}
res = requests.put(url=URL + 'position/', data=json.dumps(data), headers=headers)
print(res.status_code, res.text)
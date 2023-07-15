import pandas as pd

data = {'6019': {'range': 1.11,'dbm': -63.94}, '6018': {'range': 1.56, 'dbm': -70.56}, 'temperature': -127.00}

df = pd.DataFrame(data)

df.to_csv('test.csv', mode = 'a', index_label ='Key', header= True)
print("Success!")

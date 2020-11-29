import json


# data = {}
# data['people'] = []
# data['people'].append({
#     'name': 'Scott',
#     'website': 'stackabuse.com',
#     'from': 'Nebraska'
# })
# data['people'].append({
#     'name': 'Larry',
#     'website': 'google.com',
#     'from': 'Michigan'
# })
# data['people'].append({
#     'name': 'Tim',
#     'website': 'apple.com',
#     'from': 'Alabama'
# })

with open(r'C:\Users\aminb\Desktop\FIBO\Image\Moduel_image\parameter.json') as json_file:
    data = json.load(json_file)
    Parametersy = (data['Parameter'][0])
    print(Parametersy)
    
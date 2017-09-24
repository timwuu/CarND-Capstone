def getKey(x,y):
    return round(x/50.0)*50*10000+round(y/50.0)*50

maps_x=[5201,6402,7502,5252,5202,5199]
maps_y=[4002,2303,7891,3310,4003,3999]

map_index={}

k = len(maps_x)

for i in range(0,k):
    x = maps_x[i]
    y = maps_y[i]
    key = getKey(x,y)
    try:
        map_index[key].append(i)
    except:
        map_index[key]=[]
        map_index[key].append(i)

print(map_index)
print(map_index[getKey(5200,4000)])

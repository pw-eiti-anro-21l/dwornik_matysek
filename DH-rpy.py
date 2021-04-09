from math import sin, cos,pi
#roll - obrót wokół osi x
#pitch - obrót wokół osi y
#yaw - obrót wokół osi z
polozenie=[0,0,0]
def dh_to_rpy(lista, liczba_wierszy):
  link=[]
  for i in range(liczba_wierszy):
    link.append([])
  joint=[]
  for i in range(liczba_wierszy):
    joint.append([])
  linkxyz=[]
  for i in range(liczba_wierszy):
    linkxyz.append([])
  jointxyz=[]
  for i in range(liczba_wierszy):
    jointxyz.append([])
  for i in range(liczba_wierszy):
    a = lista[i][0]
    d = lista[i][1]
    alfa = lista[i][2]
    theta = lista[i][3]
    if a>0:
      link[i]=[0,pi/2,0]
      linkxyz[i]=[a/2,0,0]
      jointxyz[i]=[a,0,0]
    if alfa>0:
      joint[i]=[0,alfa,0]
  print(f'link rpy {link}')
  print(f'link xyz { linkxyz}')
  print(f'joint rpy {joint}')
  print(f'joint xyz {jointxyz}')

dh_to_rpy([[0,0,0,0],[0.8,0,0,0],[0.8,0,pi,0]],3)

  
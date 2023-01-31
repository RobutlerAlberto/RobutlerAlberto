import math

def vector_modulus(v1,v2):

    mod = math.sqrt(((v1[0]-v2[0])**2+v1[1]-v2[1])**2)

    return mod

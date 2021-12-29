""" try:
        alpha=math.degrees(math.atan(y/z))
        phi = math.degrees(math.atan(x / z))
        #r=math.sqrt(z**2+x**2)
    except ZeroDivisionError:
        alpha=90
        phi=90
        #r=1

    r=math.sqrt(z**2+x**2)
    w=r*np.cos(math.radians(phi))
    #r0=math.sqrt(h2**2-l1**2)
    h=math.sqrt(y**2+z**2)

    P=(l1**2+h**2-w**2)/(2*l1*h)
    Q=(l2**2 + r**2-l3**2)/(2*l2*r)
    R=(r**2 -l2**2-l3**2)/(2*l2*l3)
    theta1=alpha+math.degrees(math.atan2(P,math.sqrt(1-P**2)))
    theta2=phi+math.degrees(math.atan2(Q,math.sqrt(1-Q**2)))
    theta3=math.degrees(math.atan2(math.sqrt(1-R**2),R))      """
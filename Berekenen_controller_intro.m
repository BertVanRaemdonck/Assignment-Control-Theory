syms k z Kp Ki Ts Kd
F = Kp + Ki * (Ts*(z+1)/(2*z-2)) + Kd*((2*z-2)/(Ts*(z+1)));
iztrans(F, z, k)
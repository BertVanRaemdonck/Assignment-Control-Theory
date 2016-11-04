close all
clear all
clc
clear

%% Parameters
fs = 100;   % Hz
Ts = 1/fs;  % s

%% Viewing data
rec_random1 = readlog('log_gpio_random1.xml');

t = rec_random1.getData('time');
v = rec_random1.getData('Voltage');
enc1 = rec_random1.getData('Encoder1');
enc2 = rec_random1.getData('Encoder2');

figure()
subplot(3,1,1)
plot(t, v);
subplot(3,1,2)
plot(t, enc1);
subplot(3,1,3)
plot(t, enc2);



%% Unwrapping the values of the encoder

tol_encoder = 32761;
random1_enc1_a = enc1;
for i = length(enc1)-1:-1:1
    
    Y = sign(enc1);
    if Y(i)==1 && Y(i+1)==-1
        if enc1(i) > 32700
            random1_enc1_a(i+1:end) = random1_enc1_a(i+1:end) + (2*tol_encoder);
        end
    end
    if Y(i)==-1 && Y(i+1)==1
        if enc1(i) < -32700
            random1_enc1_a(i+1:end) = random1_enc1_a(i+1:end) - (2*tol_encoder);
        end
    end
end

figure()
subplot(3,1,1)
plot(t, random1_enc1_a);
subplot(3,1,2)
plot(t, enc1);
subplot(3,1,3)
plot(t, random1_enc1_a-enc1);


random1_enc2_a = enc2;
for i = length(enc2)-1:-1:1
    
    Y = sign(enc2);
    if Y(i)==1 && Y(i+1)==-1
        if enc2(i) > 32700
            random1_enc2_a(i+1:end) = random1_enc2_a(i+1:end) + (2*tol_encoder);
        end
    end
    if Y(i)==-1 && Y(i+1)==1
        if enc2(i) < -32700
            random1_enc2_a(i+1:end) = random1_enc2_a(i+1:end) - (2*tol_encoder);
        end
    end
end

figure()
subplot(3,1,1)
plot(t, random1_enc2_a);
subplot(3,1,2)
plot(t, enc2);
subplot(3,1,3)
plot(t, random1_enc2_a-enc2);

%% Converting to frequency domain
n = size(t,1);
f = fs*(-n/2:n/2-1)/n;
v_f = fftshift(fft(v))/n;
enc1_f = fftshift(fft(enc1))/n;

figure()
subplot(2,1,1)
plot(f, abs(v_f))
subplot(2,1,2)
plot(f, unwrap(angle(v_f)))

figure()
subplot(2,1,1)
semilogx(f, 20*log10(abs(enc1_f./v_f)))
subplot(2,1,2)
semilogx(f, 180/pi*unwrap(angle(enc1_f./v_f)))
        
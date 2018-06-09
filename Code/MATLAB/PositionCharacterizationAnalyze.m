clear angle;

%% Inputs
current = current2(321:2364);
position = desPos2(321:2364);
n = 3;  % Number of values to use

%% Perform fft on the data and extract out n highest magnitude frequencies
X = current;
Y = fft(X - mean(X));
figure; plot(abs(Y) + mean(X));

y = zeros(size(Y));
[~, bi] = sort(Y(1:end/2+1),'descend');
indices = bi(1:n);

k = 1;
for i = indices
    y(i) = 2*Y(i);
    k = k + 1;
end

% Verify the fit
figure; plot(position,X); hold on; plot(position,real(ifft(y))+mean(X)); title('Verify fit')

%% Generate the equation for the fit
L = max(size(position));
D = max(position) - min(position);
dp = D/L;
Fs = 1/dp;

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

f = Fs*(0:(L/2))/L;
figure; plot(f,P1)
title('Single-Sided Amplitude Spectrum of X(t)');
xlabel('f (1/[])');
ylabel('|P1(f)|');

syms fit x
fit(x) = mean(X);
for i = 1:max(size(indices))
    fit(x) = fit(x) + P1(indices(i))*cos(2*pi*f(indices(i))*x+angle(y(indices(i))));
end

%% Print the equation
fprintf('Decogging Equation: ');
disp(vpa(fit(x-position(1))-mean(X), 6))

clc
clear all
close all
% Electric Motor Control 
% G(s) as for plant: = 4 / (s(s+0.5)) 
tfolp = tf([4], [1, 0.5, 0]); % tf of plant 
roots([1, 0.5, 0])
tfclp = tf([4], [1, 0.5, 4])
% tfclptest = tf([1], [1, 0.5, 4]); % tf of plant 
CLPoles = roots([1, 0.5, 4]) 
fprintf('Attenuation Term: %f', real(CLPoles))
fprintf('\nDamped natural frequenciy: %f', imag(CLPoles))
wn = 1.9997; 
zeta = 0.1250; 
static_vel_e_c = 8; % static velocity error constant 
sinfoplant = stepinfo(tfclp)
step(tfclp)
% hold on 
% step(tfclptest)
wnd = 5; % desired wn  
zetad = 0.5; % desired zeta 
atted = zetad*wnd % desired attenuation term 
wdd = wnd*sqrt(1-zetad^2) % desired damped natural freq. 
S1 = -zeta*wn + 1j*wn*sqrt(1-zeta^2)
S2 = -zeta*wn - 1j*wn*sqrt(1-zeta^2)  
S1d = -zetad*wnd + 1j*wnd*sqrt(1-zetad^2)
S2d = -zetad*wnd - 1j*wnd*sqrt(1-zetad^2)
selpar = [] 
% find angle deficiency first: use evalfr or freqresp builtin fcns 
evpd = evalfr(tfolp, S1d) % ev plant desired on OPEN LOOP TF - in other words: it should be evaluated on the Plant TF
angledegevpd = rad2deg(angle(evpd))% angle in degrees of plant desired 
magevpd = abs(evpd)
angledef = 180 - angledegevpd % angle deficiency of plant at deisred cl poles loci 
if angledef > 0
    disp('you need to ADD phase -> Lead Compensator should contribute for this angle deficiency')
elseif angledef < 0
    disp('You need to Subtract phase -> Lag Compensator')
end
% let's choose a in Glc to cancel the stable pole of plant - this is
% zero-pole cancellation method!!! other one is Geometry method and method
% III is my metohd: quadratic optimization given a range of values  
% a = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9]; % zero of the lead com 
a = [0.3, 0.5, 0.7]; % zero of the lead com 
lcnangle = rad2deg(angle(S1d+a)) % lead com numerator angle 
%lcnangle = rad2deg(atan(imag(S1d+a))/real(S1d+a)) 
lcdangle = lcnangle - angledef
b = imag(S1d) ./ (tan(deg2rad(lcdangle)))  + 2.5 ; % or instead of 2.5 we'd say: imag(S1d)  

for i = 1 : length(a)
    for j = 1 : length(b)
        tflc = tf([1, a(i)], [1, b(j)]) % tf of lead compensators 
maglcalone = abs(evalfr(tflc, S1d)) % Sanity check 
anglecalone = rad2deg(angle(evalfr(tflc, S1d))) % Sanity check 

tfplc = tfolp*tflc  % tf of plant and lead comp
kc = 1 / abs(evalfr(tfplc, S1d)) % compensator gain to satisfy magnitude condition 
% use static velocity error constant to find the ratio between lagc zero
% and pole 
lagczpr = 16 % lag comp. zero pole ratio given 80 inverse second as the static velocity error constant  
% now check if magnitude and angle conditions are all met / satisfied 
maglcp = abs(evalfr(tfplc, S1d)) % Sanity check 
anglelcp = rad2deg(angle(evalfr(tfplc, S1d))) ; % sanity checks 
   % end
%end

%c = [0.1*0.2941, 0.3*0.2941, 0.4*0.2941, 0.6*0.2941, 0.8*0.2941, ...
   % 0.2941, 1.25*0.2941, 1.5*0.2941, 2*0.2941];
c = [ 0.8*0.2941, ...
    0.2941, 1.5*0.2941];
% c(i) = lagczpr .* d(i)
% c(i) = 1;   
% c = 0.2941
d = c ./ lagczpr % lead-lag compensator zero pole ratio 
for ii = 1 : length(c)
    for jj = 1 : length(d)
        tflagc = tf([1, c(ii)], [1, d(jj)])
% sanity checks on lag comp to ensure that angle contribution and magnitude conditions are all met 
maglagc = abs(evalfr(tflagc, S1d))
anglelagc = rad2deg(angle(evalfr(tflagc, S1d)))
if anglelagc > -5 && anglelagc < 0
    disp('Angle Condition is satisfied and lag compensator angle contribution is minimum')
else
    disp(['Angle condition is violated by lag compensator and FAILURE!!! and Re-design...' ...
        'zero and pole of the Lag Compensator'])
end
if maglagc > 0.97 && maglagc < 1.3
    disp(['Magnitude Condition is satisfied and lag compensator magnitude ...' ...
        'contribution is minimum'])
else
    disp(['Magnitude condition is violated by lag compensator and FAILURE!!! and Re-design...' ...
        'zero and pole of the Lag Compensator'])
end

tfolpleadc = tfolp .* tflc
tfolpleadclagc = tfolp .* kc .* tflc .* tflagc
tfolplagc = tfolp .* tflagc
tfclp = feedback(tfolp, 1, -1) % cl plant 
tfclpleadc = feedback(tfolpleadc, 1, -1) 
tfclplagc = feedback(tfolplagc, 1, -1) 
tfclpleadclagc = feedback(tfolpleadclagc, 1, -1) 

[ytfclpleadclagc, tstep] = step(tfclpleadclagc)
esstfclpleadclagc = abs(max(ytfclpleadclagc) - 1) .* 100; % steady-state error / offset 
% let's only focus on the lead lag compensator only 
tfclpleadclagcinfo = stepinfo(tfclpleadclagc)
tstfclpleadclagc = tfclpleadclagcinfo.SettlingTime 
Mptfclpleadclagc = tfclpleadclagcinfo.Overshoot
if esstfclpleadclagc < 50 && Mptfclpleadclagc < 60
    asel(i) = a(i)% a selected; note here we need to make a cell array or double array as a is with counter otherwise it'll over-write  
    bsel(j) = b(j)
    csel(ii) = c(ii)
    dsel(jj) = d(jj)
    kcsel = kc % does not have counter ... oops so 
    selpar = [selpar; asel(i), bsel(j), csel(ii), dsel(jj)] % selected parameters 
else
    agar = a % garbage a 
    bgar = b % garbage b 
    cgar = c % garbage c 
    dgar = d % garbage d 
    kcgar = kc % garbage c 

end
end
end
    end
end

%% Quadratic Optimization 
alpha = 1; % weigthing parameter for ess
beta = 1.5; % weghting parameter for Mp 
for i = 1 : size(asel)% rows(selpar)
    tfolllc(i) = (tf([1, asel(i)], [1, bsel(i)])) * (tf([1, csel(i)], [1, dsel(i)]))
    maglcaloneopti(i) = abs(evalfr(tfolllc(i), S1d)) % Sanity check 
% anglecaloneopti = rad2deg(angle(evalfr(tfolpllc{i}, S1d))) % Sanity check 

tfolpllc(i) = tfolp*tfolllc(i)  % tf of plant and lead comp
kc(i) = 1 ./ abs(evalfr(tfolpllc(i), S1d)) % compensator gain to satisfy magnitude condition 
    tfclpllc(i) = feedback(kc(i)*tfolpllc(i), 1, -1);% tf closed-loop plant lead lag compensator 
[ytfclpllcopti{i}, ttfclpllcopti{i}] = step(tfclpllc(i)); 
    esstfclpllcopti{i} = abs(max(ytfclpllcopti{i}) - 1) * 100; 
    tfclpllcoptiinfo(i) = stepinfo(tfclpllc(i));
    tstfclpllc(i) = tfclpllcoptiinfo(i).SettlingTime 
    Mptfclpllc(i) = tfclpllcoptiinfo(i).Overshoot 

    Jobj(i) = alpha*esstfclpllcopti{i}^2 + beta*Mptfclpllc(i)^2 % Objective / cost/loss function / Performance Index 
    % Jobjmatrix = cell2mat(Jobj);
    [minobjval, minobjindex] = min(Jobj)
    
    optia = selpar(minobjindex, 1)
    optib = selpar(minobjindex, 2)
    optic = selpar(minobjindex, 3)
    optid = selpar(minobjindex, 4)
    optikc = kc(minobjindex)

end
 % ver 
 % whos ycltfopti

 disp(["Optimum a (lead compensator zero) from the given set is: optia=", num2str(optia)])
  disp(["Optimum b (lead compensator pole) from the given set is: optib=", num2str(optib)])
    disp(["Optimum c (lag compensator zero) from the given set is: optic=", num2str(optic)])
      disp(["Optimum d (lag compensator pole) from the given set is: optid=", num2str(optid)])
         disp(["Optimum kc (compensator gain) from the given set is: optikc=", num2str(optikc)])

         % plots for initial values and NOT optimal values yet 
figure(3) 
step(tfclp)
grid on 
hold on 
step(tfclpleadc)
hold on 
step(tfclplagc)
hold on 
step(tfclpleadclagc)
hold off 
% legend({'Plant (G_p)', 'Plant and Lead Compensator (G_p * G_LeadC)', ...
%     'Plant and Lag Compensator (G_p * G_LagC)', ...
%     'Plant and Lag-Lead Compensator (G_p * G_LeadC * G_LagC)'}, 'FontSize', 15)
[yp, tp] = step(tfclp, 10);
e_ss_p = yp(end) - 1
[ypleadc, tpleadc] = step(tfclpleadc, 10);
e_ss_pleadc = ypleadc(end) - 1
[yplagc, tplagc] = step(tfclplagc, 10);
e_ss_plagc = yplagc(end) - 1
[ypleadclagc, tpleadlagc] = step(tfclpleadclagc, 10);
e_ss_pleadclagc = ypleadclagc(end) - 1
e_ss_pPer = abs(e_ss_p)*100
e_ss_pleadcPer = abs(e_ss_pleadc)*100
e_ss_plagcPer = abs(e_ss_plagc)*100
e_ss_pleadclagcPer = abs(e_ss_pleadclagc)*100

Sinfo1 = stepinfo(tfclp)
Sinfo2 = stepinfo(tfclpleadc)
Sinfo3 = stepinfo(tfclplagc)
Sinfo4 = stepinfo(tfclpleadclagc)

figure(4) 
rlocus(tfolp)
grid on 
hold on 
rlocus(tfolpleadc)
hold on 
rlocus(tfolplagc)
hold on 
rlocus(tfolpleadclagc)
hold off 
% legend({'Plant (G_p)', 'Plant and Lead Compensator (G_p * G_LeadC)',...
%     'Plant and Lag Compensator (G_p * G_LagC)', ...
%     'Plant and Lag-Lead Compensator (G_p * G_LeadC * G_LagC)'}, 'FontSize', 15)

figure(5) 
bode(tfclp) % Why boe of: CL? I prefer OL 
% reason is plant is Electric motor not mass-spring-damper 
grid on 
hold on 
bode(tfclpleadc)
hold on 
bode(tfclplagc)
hold on 
bode(tfclpleadclagc)
hold off 
% bode(tfolp) % Why boe of: CL? I prefer OL 
% grid on 
% hold on 
% bode(tfolpleadc)
% hold on 
% bode(tfolplagc)
% hold on 
% bode(tfolpleadclagc)
% hold off 
% legend({'Plant (G_p', 'Plant and Lead Compensator (G_p * G_LeadC)',...
%     'Plant and Lag Compensator (G_p * G_LagC)', ...
%     'Plant and Lag-Lead Compensator (G_p * G_LeadC * G_LagC)'}, 'FontSize', 15)

% Stability Margins 
[Gmp, Pmp, Wgcp, Wpcp] = margin(tfclp)

% side notes: looks like after deleting all counters (i, j, ii, jj) in the
% loops the issue is gone. only keep the counters for a,b ,c ,d  and kc is
% autmatically updated throughtout each iteration as it is dependant on a
% and b values so no more actions is required for that ... 

%% Now all results with optimal values of lead lag zeros and poles 
% offset - steady-state error calculation here 
tflcopti = tf([1, optia], [1, optib])
tflagcopti = tf([1, optic], [1, optid])
tfolpleadcopti = tfolp .* tflcopti
tfolpleadclagcopti = tfolp .* kc .* tflcopti .* tflagcopti
tfolplagcopti = tfolp .* tflagcopti
tfclpopti = feedback(tfolp, 1, -1) % cl plant 
tfclpleadcopti = feedback(tfolpleadcopti, 1, -1) 
tfclplagcopti = feedback(tfolplagcopti, 1, -1) 
tfclpleadclagcopti = feedback(tfolpleadclagcopti, 1, -1) 
figure(6) 
step(tfclp)
grid on 
hold on 
step(tfclpleadcopti)
hold on 
step(tfclplagcopti)
hold on 
step(tfclpleadclagcopti)
hold off 
% legend({'Plant (G_p', 'Plant and Optimized Lead Compensator (G_p * Optimized_G_LeadC)',...
%     'Plant and Optimized Lag Compensator (G_p * G_LagC)', ...
%     'Plant and Optimized Lag-Lead Compensator (G_p * Optimized_G_LeadC * Optimized_G_LagC)'}, 'FontSize', 12)
% legend({'Plant (G_p', 'Plant and Lead Compensator (G_p * G_LeadC)',...
%     'Plant and Lag Compensator (G_p * G_LagC)', ...
%     'Plant and Lag-Lead Compensator (G_p * G_LeadC * G_LagC)'}, 'FontSize', 15)

[yp, tp] = step(tfclp, 10);
e_ss_p = yp(end) - 1
[ypleadcopti, tpleadcopti] = step(tfclpleadcopti, 10);
e_ss_pleadcopti = ypleadcopti(end) - 1
[yplagcopti, tplagcopti] = step(tfclplagcopti, 10);
e_ss_plagcopti = yplagcopti(end) - 1
[ypleadclagcopti, tpleadlagcopti] = step(tfclpleadclagcopti, 10);
e_ss_pleadclagcopti = ypleadclagcopti(end) - 1
e_ss_pleadclagc = ypleadclagc(end) - 1
% e_ss_pPer = abs(e_ss_p)*100
e_ss_pleadcoptiPer = abs(e_ss_pleadcopti)*100
e_ss_plagcoptiPer = abs(e_ss_plagcopti)*100
e_ss_pleadclagcoptiPer = abs(e_ss_pleadclagcopti)*100

Sinfoopti1 = stepinfo(tfclp)
Sinfoopti2 = stepinfo(tfclpleadcopti)
Sinfoopti3 = stepinfo(tfclplagcopti)
Sinfoopti4 = stepinfo(tfclpleadclagcopti)

figure(7) 
rlocus(tfolp)
grid on 
hold on 
rlocus(tfolpleadcopti)
hold on 
rlocus(tfolplagcopti)
hold on 
rlocus(tfolpleadclagcopti)
hold off 
% legend({'Plant (G_p', 'Plant and Optimized Lead Compensator (G_p * Optimized_G_LeadC)',...
%     'Plant and Optimized Lag Compensator (G_p * G_LagC)', ...
%     'Plant and Optimized Lag-Lead Compensator (G_p * Optimized_G_LeadC * Optimized_G_LagC)'}, 'FontSize', 12)
% legend({'Plant (G_p)', 'Plant and Lead Compensator (G_p * G_LeadC)',...
%     'Plant and Lag Compensator (G_p * G_LagC)', ...
%     'Plant and Lag-Lead Compensator (G_p * G_LeadC * G_LagC)'}, 'FontSize', 15)

figure(8) 
bode(tfclp)
grid on 
hold on 
bode(tfclpleadcopti)
hold on 
bode(tfclplagcopti)
hold on 
bode(tfclpleadclagcopti)
hold off 
% legend({'Plant (G_p', 'Plant and Optimized Lead Compensator (G_p * Optimized_G_LeadC)',...
%     'Plant and Optimized Lag Compensator (G_p * G_LagC)', ...
%     'Plant and Optimized Lag-Lead Compensator (G_p * Optimized_G_LeadC * Optimized_G_LagC)'}, 'FontSize', 12)

% Stability Margins 
% [Gmp, Pmp, Wgcp, Wpcp] = margin(tfclp)


%% for step, impulse, ramp responses use CL TFs 
%% for bode, nyquist, nichols, freqresp, and rootlocus; use OL TF
%% but it depedns in this example we used bode for cl 
% mtest1 = 1; 
% ktest1 = 1e6;
% zetatest1 = 0.05;
% ccrtest1 = 2 * sqrt(mtest1 * ktest1)
% ctest1 = zetatest1 * ccrtest1 
% 
% tfmsdtest1 = tf([1], [mtest1, ctest1, ktest1])
% figure(6)
% bode(tfmsdtest1, '.-')
% grid on 
% hold on 
% bode(feedback(tfmsdtest1, 1, -1), '-')


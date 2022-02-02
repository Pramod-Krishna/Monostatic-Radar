load BasicMonostaticRadarExampleData;
prop_speed = radiator.PropagationSpeed;
max_range = 8000;
prf = prop_speed/(2*max_range);
waveform.PRF = prf;
num_pulse_int = 10;
pfa = 1e-6;
snr_min = shnidman(0.9,pfa,num_pulse_int,2)
fc = radiator.OperatingFrequency;
lambda = prop_speed/fc;
peak_power = ((4*pi)^3*noisepow(1/waveform.PulseWidth)*max_range^4*...
    db2pow(snr_min))/(db2pow(2*transmitter.Gain)*1*lambda^2)
range_res = 50; 
pulse_bw = prop_speed/(2*range_res);
pulse_width = 20/pulse_bw;
fs = 2*pulse_bw;

waveform = phased.LinearFMWaveform(...
    'SweepBandwidth',pulse_bw,...
    'PulseWidth',pulse_width,...
    'PRF',prf,...
    'SampleRate',fs);
peak_power = ((4*pi)^3*noisepow(1/waveform.PulseWidth)*max_range^4*...
    db2pow(snr_min))/(db2pow(2*transmitter.Gain)*1*lambda^2)
transmitter.PeakPower = peak_power;
tgtpos = [[2024.66; 0; 0],[6518.63; 0; 0],[6845.04; 0; 0]];
tgtvel = [[0;0;0],[0;0;0],[0;0;0]];
tgtmotion = phased.Platform('InitialPosition',tgtpos,'Velocity',tgtvel);

tgtrcs = [2.2 1.1 1.05];
fc = radiator.OperatingFrequency;
target = phased.RadarTarget(...
    'Model','Swerling2',...
    'MeanRCS',tgtrcs,...
    'OperatingFrequency',fc);
target.SeedSource = 'Property';
target.Seed = 2007;
channel = phased.FreeSpace(...
    'SampleRate',waveform.SampleRate,...
    'TwoWayPropagation',true,...
    'OperatingFrequency',fc);
receiver.SeedSource = 'Property';
receiver.Seed = 2007;

fast_time_grid = unigrid(0,1/fs,1/prf,'[)');
slow_time_grid = (0:num_pulse_int-1)/prf;

rxpulses = zeros(numel(fast_time_grid),num_pulse_int); % pre-allocate 

for m = 1:num_pulse_int
    
    % Update sensor and target positions
    [sensorpos,sensorvel] = sensormotion(1/prf);
    [tgtpos,tgtvel] = tgtmotion(1/prf);

    % Calculate the target angles as seen by the sensor
    [tgtrng,tgtang] = rangeangle(tgtpos,sensorpos);
    
    % Simulate propagation of pulse in direction of targets
    pulse = waveform();
    [txsig,txstatus] = transmitter(pulse);
    txsig = radiator(txsig,tgtang);
    txsig = channel(txsig,sensorpos,tgtpos,sensorvel,tgtvel);
    
    % Reflect pulse off of targets
    tgtsig = target(txsig,true);
    
    % Receive target returns at sensor
    rxsig = collector(tgtsig,tgtang);
    rxpulses(:,m) = receiver(rxsig,~(txstatus>0));
end
noise_bw = receiver.SampleRate/2;
npower = noisepow(noise_bw,...
    receiver.NoiseFigure,receiver.ReferenceTemperature);
threshold = npower * db2pow(npwgnthresh(pfa,num_pulse_int,'noncoherent'));

pulseplotnum = 2;
helperRadarPulsePlot(rxpulses,threshold,...
    fast_time_grid,slow_time_grid,pulseplotnum);
matchingcoeff = getMatchedFilter(waveform);
matchedfilter = phased.MatchedFilter(...
    'CoefficientsSource','Property',...
    'Coefficients',matchingcoeff,...
    'GainOutputPort',true);
[rxpulses, mfgain] = matchedfilter(rxpulses);
threshold = threshold * db2pow(mfgain);
matchingdelay = size(matchingcoeff,1)-1;
rxpulses = buffer(rxpulses(matchingdelay+1:end),size(rxpulses,1));
range_gates = prop_speed*fast_time_grid/2; 
lambda = prop_speed/fc;

tvg = phased.TimeVaryingGain(...
    'RangeLoss',2*fspl(range_gates,lambda),...
    'ReferenceLoss',2*fspl(max_range,lambda));
rxpulses = tvg(rxpulses);
rxpulses = pulsint(rxpulses,'noncoherent');

helperRadarPulsePlot(rxpulses,threshold,fast_time_grid,slow_time_grid,1);
[~,range_detect] = findpeaks(rxpulses,'MinPeakHeight',sqrt(threshold));

true_range = round(tgtrng)
range_estimates = round(range_gates(range_detect))

pd = 0.9;            % Probability of detection
pfa = 1e-6;          % Probability of false alarm
max_range = 5000;    % Maximum unambiguous range
range_res = 50;      % Required range resolution
tgt_rcs = 1;         % Required target radar cross section
prop_speed = physconst('LightSpeed');   % Propagation speed
pulse_bw = prop_speed/(2*range_res);    % Pulse bandwidth
pulse_width = 1/pulse_bw;               % Pulse width
prf = prop_speed/(2*max_range);         % Pulse repetition frequency
fs = 2*pulse_bw;                        % Sampling rate
waveform = phased.RectangularWaveform(...
    'PulseWidth',1/pulse_bw,...
    'PRF',prf,...
    'SampleRate',fs);

noise_bw = pulse_bw;

receiver = phased.ReceiverPreamp(...
    'Gain',20,...
    'NoiseFigure',0,...
    'SampleRate',fs,...
    'EnableInputPort',true);

num_pulse_int = 10;
rocsnr([0 3 5],'SignalType','NonfluctuatingNoncoherent',...
    'NumPulses',num_pulse_int);

snr_min = albersheim(pd, pfa, num_pulse_int)
tx_gain = 20;

fc = 10e9;
lambda = prop_speed/fc;

peak_power = ((4*pi)^3*noisepow(1/pulse_width)*max_range^4*...
    db2pow(snr_min))/(db2pow(2*tx_gain)*tgt_rcs*lambda^2)


transmitter = phased.Transmitter(...
    'Gain',tx_gain,...
    'PeakPower',peak_power,...
    'InUseOutputPort',true);
antenna = phased.IsotropicAntennaElement(...
    'FrequencyRange',[5e9 15e9]);

sensormotion = phased.Platform(...
    'InitialPosition',[0; 0; 0],...
    'Velocity',[0; 0; 0]);
radiator = phased.Radiator(...
    'Sensor',antenna,...
    'OperatingFrequency',fc);

collector = phased.Collector(...
    'Sensor',antenna,...
    'OperatingFrequency',fc);
tgtpos = [[2024.66;0;0],[3518.63;0;0],[3845.04;0;0]];
tgtvel = [[0;0;0],[0;0;0],[0;0;0]];
tgtmotion = phased.Platform('InitialPosition',tgtpos,'Velocity',tgtvel);

tgtrcs = [1.6 2.2 1.05];
target = phased.RadarTarget('MeanRCS',tgtrcs,'OperatingFrequency',fc);
channel = phased.FreeSpace(...
    'SampleRate',fs,...
    'TwoWayPropagation',true,...
    'OperatingFrequency',fc);
fast_time_grid = unigrid(0,1/fs,1/prf,'[)');
slow_time_grid = (0:num_pulse_int-1)/prf;
receiver.SeedSource = 'Property';
receiver.Seed = 2007;

% Pre-allocate array for improved processing speed
rxpulses = zeros(numel(fast_time_grid),num_pulse_int);

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
    tgtsig = target(txsig);

    % Receive target returns at sensor
    rxsig = collector(tgtsig,tgtang);
    rxpulses(:,m) = receiver(rxsig,~(txstatus>0));
end

npower = noisepow(noise_bw,receiver.NoiseFigure,...
    receiver.ReferenceTemperature);
threshold = npower * db2pow(npwgnthresh(pfa,num_pulse_int,'noncoherent'));

num_pulse_plot = 2;
helperRadarPulsePlot(rxpulses,threshold,...
    fast_time_grid,slow_time_grid,num_pulse_plot);

matchingcoeff = getMatchedFilter(waveform);
matchedfilter = phased.MatchedFilter(...
    'Coefficients',matchingcoeff,...
    'GainOutputPort',true);
[rxpulses, mfgain] = matchedfilter(rxpulses);

matchingdelay = size(matchingcoeff,1)-1;
rxpulses = buffer(rxpulses(matchingdelay+1:end),size(rxpulses,1));

threshold = threshold * db2pow(mfgain);

helperRadarPulsePlot(rxpulses,threshold,...
    fast_time_grid,slow_time_grid,num_pulse_plot);

range_gates = prop_speed*fast_time_grid/2;

tvg = phased.TimeVaryingGain(...
    'RangeLoss',2*fspl(range_gates,lambda),...
    'ReferenceLoss',2*fspl(max_range,lambda));

rxpulses = tvg(rxpulses);

helperRadarPulsePlot(rxpulses,threshold,...
    fast_time_grid,slow_time_grid,num_pulse_plot);

rxpulses = pulsint(rxpulses,'noncoherent');

helperRadarPulsePlot(rxpulses,threshold,...
    fast_time_grid,slow_time_grid,1);
[~,range_detect] = findpeaks(rxpulses,'MinPeakHeight',sqrt(threshold));

true_range = round(tgtrng)
range_estimates = round(range_gates(range_detect))



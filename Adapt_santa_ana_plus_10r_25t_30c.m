function Adapt_santa_ana_plus_10r_25t_30c
scenario = load('output_santa_ana_plus_10r_25t_30c');
scenario.n_vehicle = 18000;

spec = EAMoDspec.CreateFromScenarioAE(scenario);

save('EAMoDspecDemo_10r_25t_30c','spec');

end
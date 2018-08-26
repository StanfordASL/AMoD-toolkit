function aaCreateDemoEAMoDspec

data_path = 'Test/dfw_roadgraph_kmeans_tv_federico_5cl_windsun_12h_v3_feas';
scenario = LoadScenario(data_path);
spec = EAMoDspec.CreateFromScenario(scenario);
save('EAMoDspecDemo.mat','spec')

end
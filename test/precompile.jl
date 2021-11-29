using PackageCompiler


# for PILS example
create_sysimage(
		[
		 "PyCall",
		 "FlightSims",
		 "FSimBase",
		 "FSimZoo",
		 "FSimROS",
		 "FSimPlots",
		 "Plots",
		 "UnPack",
		 "OnlineStats",
		 "DataFrames",
		 "ComponentArrays",
		 ],
		sysimage_path="sys_pils.so",
		precompile_execution_file="test/PILS/precompile_execution.jl",
	       )

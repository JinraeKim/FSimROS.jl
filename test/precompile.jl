using PackageCompiler


# for PILS example
create_sysimage(
		[
		 "PyCall",
		 "FlightSims",
		 "FSimROS",
		 "Plots",
		 "FSimPlots",
		 "UnPack",
		 "OnlineStats",
		 "DataFrames",
		 "ComponentArrays",
		 ],
		sysimage_path="sys_pils.so",
		precompile_execution_file="test/PILS/precompile_execution.jl",
	       )

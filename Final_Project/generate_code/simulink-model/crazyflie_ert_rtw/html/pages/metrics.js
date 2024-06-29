function CodeMetrics() {
	 this.metricsArray = {};
	 this.metricsArray.var = new Array();
	 this.metricsArray.fcn = new Array();
	 this.metricsArray.var["crazyflie.c:crazyflie_M_"] = {file: "D:\\Chalmers\\Sp4\\SSY191\\BitcrazeSharedFolder\\test\\project2024-13group\\generate_code\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	size: 2};
	 this.metricsArray.var["crazyflie_DW"] = {file: "D:\\Chalmers\\Sp4\\SSY191\\BitcrazeSharedFolder\\test\\project2024-13group\\generate_code\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	size: 16};
	 this.metricsArray.var["crazyflie_U"] = {file: "D:\\Chalmers\\Sp4\\SSY191\\BitcrazeSharedFolder\\test\\project2024-13group\\generate_code\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	size: 88};
	 this.metricsArray.var["crazyflie_Y"] = {file: "D:\\Chalmers\\Sp4\\SSY191\\BitcrazeSharedFolder\\test\\project2024-13group\\generate_code\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	size: 56};
	 this.metricsArray.fcn["atan2"] = {file: "C:\\Program Files\\MATLAB\\R2023b\\polyspace\\verifier\\cxx\\include\\include-libc\\bits\\mathcalls.h",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["crazyflie.c:rate_scheduler"] = {file: "D:\\Chalmers\\Sp4\\SSY191\\BitcrazeSharedFolder\\test\\project2024-13group\\generate_code\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["crazyflie_initialize"] = {file: "D:\\Chalmers\\Sp4\\SSY191\\BitcrazeSharedFolder\\test\\project2024-13group\\generate_code\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["crazyflie_step"] = {file: "D:\\Chalmers\\Sp4\\SSY191\\BitcrazeSharedFolder\\test\\project2024-13group\\generate_code\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	stack: 104,
	stackTotal: 104};
	 this.metricsArray.fcn["crazyflie_terminate"] = {file: "D:\\Chalmers\\Sp4\\SSY191\\BitcrazeSharedFolder\\test\\project2024-13group\\generate_code\\simulink-model\\crazyflie_ert_rtw\\crazyflie.c",
	stack: 0,
	stackTotal: 0};
	 this.metricsArray.fcn["sqrt"] = {file: "C:\\Program Files\\MATLAB\\R2023b\\polyspace\\verifier\\cxx\\include\\include-libc\\bits\\mathcalls.h",
	stack: 0,
	stackTotal: 0};
	 this.getMetrics = function(token) { 
		 var data;
		 data = this.metricsArray.var[token];
		 if (!data) {
			 data = this.metricsArray.fcn[token];
			 if (data) data.type = "fcn";
		 } else { 
			 data.type = "var";
		 }
	 return data; }; 
	 this.codeMetricsSummary = '<a href="javascript:void(0)" onclick="return postParentWindowMessage({message:\'gotoReportPage\', pageName:\'crazyflie_metrics\'});">Global Memory: 162(bytes) Maximum Stack: 104(bytes)</a>';
	}
CodeMetrics.instance = new CodeMetrics();

# AgentDrive

##Branches
- Master - main branch with the latest stable code.
- Slim - branch intended to clean the project and make thinks simplier.
- Other branches are historical depreceated branches.

#Documentation AgentDrive
##How to run
AgentDrive - highway
MainClass: cz.agents.alite.Main
Program Arguments: cz.agents.highway.creator.DashBoardController settings/groovy/local/???.groovy

where the first argument is Creator to be used and second argument is configuration file.

If the second argument is not used the default config file is used i.e. settings/groovy/highway.groovy. It is recommended not to change the default config file if not necessary, you can create a copy of it in subfolder 'local' and use it instead.

###DashboardController
Dashboard controller allows to run only AgentDrive(highway) that will start also all configured simulators in config file. This can work under condition that there are proper scripts for running each simulator.
See config file you use (settings/groovy/highway.groovy or settings/groovy/local/????.groovy) for checking where thr scripts should be located. The script are not under version control in mercurial because these are specific for each user's environment.

There is an in-build local simulator avaible. It can be enabled from the config by uncomenting the line simulatorsToRun = [];

There is a possibility to use another simulators.
We provide example of running simulator-lite, we suppose the simulator-lite is opened in IDE as a module, than you can run the simulator from IDE and copy the run command from console. You might need to change directory first to make it work. In our case the simulator-lite is located in ../simulator-lite folder.

The example launchers/simulator-lite.sh:
```
#!/bin/bash
cd ../simulator-lite; /opt/java/jdk1.7/bin/java -Didea.launcher.port=7538 -Didea.launcher.bin.path=/opt/idea-IU-135.690/bin -Dfile.encoding=UTF-8 -classpath /opt/java/jdk1.7/jre/lib/deploy.jar:/opt/java/jdk1.7/jre/lib/resources.jar:/opt/java/jdk1.7/jre/lib/jsse.jar:/opt/java/jdk1.7/jre/lib/management-agent.jar:/opt/java/jdk1.7/jre/lib/jfr.jar:/opt/java/jdk1.7/jre/lib/plugin.jar:/opt/java/jdk1.7/jre/lib/jfxrt.jar:/opt/java/jdk1.7/jre/lib/charsets.jar:/opt/java/jdk1.7/jre/lib/jce.jar:/opt/java/jdk1.7/jre/lib/rt.jar:/opt/java/jdk1.7/jre/lib/javaws.jar:/opt/java/jdk1.7/jre/lib/ext/zipfs.jar:/opt/java/jdk1.7/jre/lib/ext/localedata.jar:/opt/java/jdk1.7/jre/lib/ext/dnsns.jar:/opt/java/jdk1.7/jre/lib/ext/sunec.jar:/opt/java/jdk1.7/jre/lib/ext/sunjce_provider.jar:/opt/java/jdk1.7/jre/lib/ext/sunpkcs11.jar:/home/martin/projects/simulator-lite/target/classes:/home/martin/.m2/repository/cz/agents/alite/alite/1.0-SNAPSHOT/alite-1.0-20140814.133609-4.jar:/home/martin/.m2/repository/log4j/log4j/1.2.16/log4j-1.2.16.jar:/home/martin/.m2/repository/java3d/vecmath/1.5.2/vecmath-1.5.2.jar:/home/martin/.m2/repository/jgrapht-jdk1/6/jgrapht-jdk1.6/0.8.2/jgrapht-jdk1.6-0.8.2.jar:/home/martin/.m2/repository/jgraph/jgraph/5.13.0.0/jgraph-5.13.0.0.jar:/home/martin/.m2/repository/cz/agents/agentdrive/protobufCommunicator/0.0.1-SNAPSHOT/protobufCommunicator-0.0.1-20140724.090507-7.jar:/home/martin/.m2/repository/com/google/protobuf/protobuf-java/2.4.1/protobuf-java-2.4.1.jar:/home/martin/.m2/repository/cz/agents/agentdrive/protobuf/0.0.1-SNAPSHOT/protobuf-0.0.1-20140714.130603-6.jar:/home/martin/projects/highway/target/classes:/home/martin/.m2/repository/ags/utils/dataStructures/kdtreerednaxela/3.0-SNAPSHOT/kdtreerednaxela-3.0-20130716.171801-1.jar:/home/martin/.m2/repository/cz/agents/alite/configreader/3.0.4/configreader-3.0.4.jar:/home/martin/.m2/repository/org/codehaus/groovy/groovy/2.2.2/groovy-2.2.2.jar:/home/martin/.m2/repository/org/ow2/asm/asm-tree/4.1/asm-tree-4.1.jar:/home/martin/.m2/repository/org/ow2/asm/asm/4.1/asm-4.1.jar:/home/martin/.m2/repository/antlr/antlr/2.7.7/antlr-2.7.7.jar:/home/martin/.m2/repository/org/ow2/asm/asm-util/4.1/asm-util-4.1.jar:/home/martin/.m2/repository/org/ow2/asm/asm-commons/4.1/asm-commons-4.1.jar:/home/martin/.m2/repository/org/ow2/asm/asm-analysis/4.1/asm-analysis-4.1.jar:/home/martin/.m2/repository/orca/orca/1.0-SNAPSHOT/orca-1.0-20140804.175541-46.jar:/home/martin/.m2/repository/cz/agents/alite/trajectorytools/2.0-SNAPSHOT/trajectorytools-2.0-20140819.151239-16.jar:/home/martin/.m2/repository/org/jscience/jscience/4.3.1/jscience-4.3.1.jar:/home/martin/.m2/repository/math/javageom/javaGeom/0.11.1/javaGeom-0.11.1.jar:/home/martin/.m2/repository/com/vividsolutions/jts/1.12/jts-1.12.jar:/home/martin/.m2/repository/org/teneighty/heaps/2.0.0/heaps-2.0.0.jar:/home/martin/.m2/repository/com/google/guava/guava/15.0/guava-15.0.jar:/home/martin/.m2/repository/org/apache/commons/commons-lang3/3.1/commons-lang3-3.1.jar:/home/martin/.m2/repository/cz/agents/alite/deconflictiontools/1.0-SNAPSHOT/deconflictiontools-1.0-20140822.153116-15.jar:/home/martin/.m2/repository/org/jgrapht/jgrapht-core/0.9.0/jgrapht-core-0.9.0.jar:/home/martin/.m2/repository/xerces/xercesImpl/2.11.0/xercesImpl-2.11.0.jar:/home/martin/.m2/repository/xml-apis/xml-apis/1.4.01/xml-apis-1.4.01.jar:/home/martin/.m2/repository/org/apache/commons/commons-math3/3.2/commons-math3-3.2.jar:/home/martin/.m2/repository/commons-io/commons-io/1.3.2/commons-io-1.3.2.jar:/opt/idea-IU-135.690/lib/idea_rt.jar com.intellij.rt.execution.application.AppMain cz.agents.agentdrive.simulator.lite.creator.SimulatorCreator cz.agents.agentdrive.simulator.lite.creator.SimulatorCreator
```

##Simulator-lite
MainClass: cz.agents.agentdrive.simulator.lite.creator.SimulatorCreator
Program Arguments: cz.agents.agentdrive.simulator.lite.creator.SimulatorCreatory
Working Directory: $MODULE_DIR$

where the first argument is Creator to be used. Working directory should be set to the module directory if you have imported simulator-lite as a module of the AgentDrive(highway) project.


##Simulator (openDS)
MainClass: eu.opends.main.Simulator
Program Arguments: _driving_task_path
Working Directory: $MODULE_DIR$

where driving_task_path can be for example assets/DrivingTasks/Projects/Highway/osm.xml

Note that you should run AgentDrive(Highway) first, then run the simulator.
If you do not use the .sh script to start the simulator you can choose Empty in AgentDrive configuration and run the simulator manually.


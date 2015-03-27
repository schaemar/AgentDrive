#!/bin/bash
#
# Helper script for running set of experiments

# NOTE: Set these variables according to your system!
#
# Home directory of the Java jdk
JAVA_JDK_HOME=$(java-config -O)
# Home of the Intellij Idea
IDEA_HOME="/opt/idea-community-13.1.3.135.909"
# Maven repository
MAVEN_REP="$HOME/.m2/repository"

MEMORY="2048m"

# Command to run the ExperimentCreator class
COMMAND="${JAVA_JDK_HOME}/bin/java -Didea.launcher.port=7533 -Didea.launcher.bin.path=${IDEA_HOME}/bin -Dfile.encoding=UTF-8 -Xmx$MEMORY -classpath /usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/resources.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/rt.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/deploy.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/charsets.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/jsse.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/javaws.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/plugin.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/jce.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/management-agent.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/jfxrt.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/jfr.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/ext/sunpkcs11.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/ext/zipfs.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/ext/sunjce_provider.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/ext/sunec.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/ext/localedata.jar:/usr/lib/jvm/oracle-jdk-bin-1.7/jre/lib/ext/dnsns.jar:/home/wmatex/Projects/Agents/highway/target/classes:/home/wmatex/.m2/repository/cz/agents/alite/alite/1.0-SNAPSHOT/alite-1.0-20140911.133610-5.jar:/home/wmatex/.m2/repository/log4j/log4j/1.2.16/log4j-1.2.16.jar:/home/wmatex/.m2/repository/java3d/vecmath/1.5.2/vecmath-1.5.2.jar:/home/wmatex/.m2/repository/jgrapht-jdk1/6/jgrapht-jdk1.6/0.8.2/jgrapht-jdk1.6-0.8.2.jar:/home/wmatex/.m2/repository/jgraph/jgraph/5.13.0.0/jgraph-5.13.0.0.jar:/home/wmatex/.m2/repository/ags/utils/dataStructures/kdtreerednaxela/3.0-SNAPSHOT/kdtreerednaxela-3.0-20130716.171801-1.jar:/home/wmatex/Projects/Agents/protobufCommunicator/target/classes:/home/wmatex/.m2/repository/com/google/protobuf/protobuf-java/2.4.1/protobuf-java-2.4.1.jar:/home/wmatex/Projects/Agents/protobuf/target/classes:/home/wmatex/.m2/repository/cz/agents/alite/configreader/3.0.4/configreader-3.0.4.jar:/home/wmatex/.m2/repository/org/codehaus/groovy/groovy/2.2.2/groovy-2.2.2.jar:/home/wmatex/.m2/repository/org/ow2/asm/asm-tree/4.1/asm-tree-4.1.jar:/home/wmatex/.m2/repository/org/ow2/asm/asm/4.1/asm-4.1.jar:/home/wmatex/.m2/repository/antlr/antlr/2.7.7/antlr-2.7.7.jar:/home/wmatex/.m2/repository/org/ow2/asm/asm-util/4.1/asm-util-4.1.jar:/home/wmatex/.m2/repository/org/ow2/asm/asm-commons/4.1/asm-commons-4.1.jar:/home/wmatex/.m2/repository/org/ow2/asm/asm-analysis/4.1/asm-analysis-4.1.jar:/home/wmatex/.m2/repository/commons-cli/commons-cli/1.2/commons-cli-1.2.jar:/home/wmatex/.m2/repository/orca/orca/1.0-SNAPSHOT/orca-1.0-20140804.175541-46.jar:/home/wmatex/.m2/repository/org/jscience/jscience/4.3.1/jscience-4.3.1.jar:/home/wmatex/.m2/repository/math/javageom/javaGeom/0.11.1/javaGeom-0.11.1.jar:/home/wmatex/.m2/repository/com/vividsolutions/jts/1.12/jts-1.12.jar:/home/wmatex/.m2/repository/org/teneighty/heaps/2.0.0/heaps-2.0.0.jar:/home/wmatex/.m2/repository/com/google/guava/guava/15.0/guava-15.0.jar:/home/wmatex/.m2/repository/org/apache/commons/commons-lang3/3.1/commons-lang3-3.1.jar:/home/wmatex/.m2/repository/cz/agents/alite/deconflictiontools/1.0-SNAPSHOT/deconflictiontools-1.0-20140930.080028-73.jar:/home/wmatex/.m2/repository/org/jgrapht/jgrapht-core/0.9.0/jgrapht-core-0.9.0.jar:/home/wmatex/.m2/repository/xerces/xercesImpl/2.11.0/xercesImpl-2.11.0.jar:/home/wmatex/.m2/repository/xml-apis/xml-apis/1.4.01/xml-apis-1.4.01.jar:/home/wmatex/.m2/repository/org/apache/commons/commons-math3/3.2/commons-math3-3.2.jar:/home/wmatex/.m2/repository/commons-io/commons-io/1.3.2/commons-io-1.3.2.jar:/home/wmatex/Projects/Agents/trajectorytools/target/classes:/opt/idea-community-13.1.3.135.909/lib/idea_rt.jar com.intellij.rt.execution.application.AppMain cz.agents.highway.experiments.ExperimentCreator"

# Run the experiment creator class
$COMMAND $@


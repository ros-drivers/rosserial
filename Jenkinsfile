pipeline {
  agent any
  environment {
    PACKAGE_NAME = 'ros_teensy'
    ROS_WORKSPACE = "${WORKSPACE}_ws"
  }
  stages {
    stage('Setup') {
      steps {
        sh 'printenv'
        sh """
          mkdir -p ${ROS_WORKSPACE}/src
          cp -R . ${ROS_WORKSPACE}/src/${PACKAGE_NAME}
        """
      }
    }
    stage('Build') {
      steps {
        dir(path: "${ROS_WORKSPACE}") {
          sh '''
            . /opt/ros/kinetic/setup.sh
            catkin build --no-status --verbose
          '''
        }
        
      }
    }
    stage('Test') {
      steps {
        dir(path: "${ROS_WORKSPACE}") {
          sh '''
            . /opt/ros/kinetic/setup.sh
            . devel/setup.sh
            catkin run_tests
            catkin_test_results build --verbose
          '''
        }
      }
    }
    stage('Lint') {
      steps {
        sh '''
          . /opt/ros/kinetic/setup.sh
          catkin lint --explain -W2 --strict .
        '''
      }
    }
  }
  post {
    always {
      dir(path: "${ROS_WORKSPACE}") {
        archiveArtifacts(artifacts: "logs/**/*.log", fingerprint: true)
        script {
          def files = findFiles glob: 'build/**/test_results/**/*.xml'
          if (files.length > 0) {
            junit 'build/**/test_results/**/*.xml'
          }
        }
      }
      sh "rm -rf ${ROS_WORKSPACE}"
    }
  }
}

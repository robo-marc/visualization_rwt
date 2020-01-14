module.exports = function (grunt) {
  var pkg = grunt.file.readJSON('package.json');
  grunt.initConfig({
    less: {
      development: {
        options: {
          paths: ['../less/']
        },
        files: {
          '../www/css/rwt_image_view_marc.css': '../less/rwt_image_view_marc.less'
        }
      },
      production: {
        options: {
          paths: ['../less/']
        },
        files: {
          '../www/css/rwt_image_view_marc.css': '../less/rwt_image_view_marc.less'
        }
      },
    },
    concat: {
      build: {
        src: ['../src/*.js'],
        dest: '../www/js/rwt_image_view_marc.js'
      }
    },
    uglify: {
      options: {
        report: 'min'
      },
      build: {
        src: '../www/js/rwt_image_view_marc.js',
        dest: '../www/js/rwt_image_view_marc.min.js'
      }
    },
    jshint: {
      options: {
        jshintrc: '.jshintrc'
      },
      files: [
        'Gruntfile.js',
        '../src/*.js',
        '../www/js/mjpegcanvasjs_viewer.js',
        '../www/js/rwt_image_view_marc_main.js'
      ]
    },
    watch: {
      files: ['../src/*.js', 'Gruntfile.js', '.jshintrc', '../less/rwt_image_view_marc.less'],
      tasks: ['build', 'less', 'doc']
    },
    jsdoc: {
      dist: {
        src: ['../src/*.js'],
        options: {
          destination: '../doc'
        }
      }
    }
  });

  // reading npm tasks
  for (var taskName in pkg.devDependencies) {
    if (taskName.substring(0, 6) === 'grunt-'.toString()) {
      grunt.loadNpmTasks(taskName);
    }
  }

  //  grunt.registerTask('build', ['concat', 'jshint', 'uglify', 'less']);
  grunt.registerTask('build', ['jshint', 'less']);
  grunt.registerTask('default', ['build']);
  grunt.registerTask('doc', ['jsdoc']);
};

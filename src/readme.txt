docs:

  really don't need most of this stuff, since this isn't going to do
  anything except run internally to the ansible / vbox and isn't doing to
  handle anything except a bunch of gets and posts as an intermediary to
  ROS and gazebo calls.

  nonetheless, the relevant doc URLS

    http://flask.pocoo.org/docs/0.11/
    http://flask.pocoo.org/docs/0.11/quickstart/#a-minimal-application
    http://blog.luisrei.com/articles/flaskrest.html



command line test examples:

  curl localhost:5000/logs/status/DASSTATUS
  curl -X POST -d "test" localhost:5000/phase1/power/stop_challenge_problem
  curl -X POST -d "test" localhost:5000/phase1/power/start_challenge_problem

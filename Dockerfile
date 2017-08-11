FROM cmu-mars/base

# add the source code for the shared "notifications" module
RUN git clone https://github.com/cmu-mars/notifications-p15 \
      --depth 1 \
      src/notifications

ADD . src/brasscomms

CMD ["python src/brasscomms/src/brasscomms.py"]

#!/usr/bin/python3

# remove-comment-from-code.py
#   removes comments and indents.
#   works as a filter and reads from stdin.
#   keeps lines with copyright or license information
#   reason: on a smal esp32 device we needed to gain space for
#           flash storage, in concrete: style.css
#   options: -d "// ^#", meaning also one-line-comments like //,
#                        and lines beginning with '#'
#
#   usage: remove-comment-from-code.py < style.css.orig > style.css
#
#   examples:
#   remove-comment-from-code.py -d "// ^#"
#       in                       out
#       -----------------------+-------------
#       foo /* bar */ abc        foo abc
#       foo // bar abc           foo
#       // bar abc               line ignored
#       /* this is a comment */  line ignored
#       foo /* this is a         foo bar
#         comment text */ bar
#       # foo                    line ignored
#       foo # bar                foo
#         foo        bar         foo bar
#
# (c) 2024-03-19 Thomas Osterried <dl9sau@darc.de>. License: GPL

import sys

one_line_comment_chars = []

def usage():
  sys.stderr.write("usage: %s [-d x]\n" % sys.argv[0])
  sys.stderr.write("       with x i.e. \"// ^#\", meaning also one-line-comments)\n")
  sys.stderr.write("       like //, and lines beginning with '#'\n")
  sys.exit(1)


def remove_tabs_and_reduce_blanks_to_one_and_strip(s):
  return s.replace("\t", " ").replace("  ", " ").replace("  ", " ").strip()


def set_one_line_comment_chars(s):
  global one_line_comment_chars
  if type(s) == list and len(s):
    one_line_comment_chars = s
  elif type(s) == str and len(s):
    one_line_comment_chars = s.split(" ")
  else:
    one_line_comment_chars = []


def do_remove_comment_from_code(file_in = None, file_out = None):
  commentStart = -1
  line_out = ""

  if not file_in:
    file_in = sys.stdin
  if not file_out:
    file_out = sys.stdout

  while True:
    line_in = file_in.readline()
    if line_in == "": break
    line_in = line_in.rstrip()

    if commentStart == -1:
      commentStart = line_in.find("/*")
    if commentStart > -1:
      if len(line_out) > 0 and not line_out[len(line_out)-1].isspace():
        line_out = line_out + " "
      line_out = line_out + line_in
      commentEnd = line_out.find("*/")
      if commentEnd == -1:
        continue
      comment = line_out[commentStart+2:commentEnd]
      comment = comment.lower()
      if comment.find("license") == -1 and comment.find("copyright") == -1:
        line_out = line_out[:commentStart] + " " + line_out[commentEnd+2:]
      # else:
        # leave comment as is
      commentStart = -1
    else:
      line_out = line_in
      for s in one_line_comment_chars:
        if s[0] == '^':
          if line_in.startswith(s[1:]):
            line_out = ""
            break
        else:
          comment_pos = line_in.find(s)
          if comment_pos > -1:
            line_out = line_in[:comment_pos]
            break

    line_out = remove_tabs_and_reduce_blanks_to_one_and_strip(line_out)
    if len(line_out) > 0:
      file_out.write(line_out + "\n")
      line_out = ""


# main()

if __name__ == '__main__':
  argc = len(sys.argv)
  if argc > 1:
    if argc > 2:
      if sys.argv[1] == "-d":
        set_one_line_comment_chars(sys.argv[2])
        # cave: not valid for css, because ^# is a valid part of syntax
        for s in one_line_comment_chars:
          if s[0] == "^" and len(s) < 2:
            usage()
          else:
            if len(s) < 1:
              usage()
      else:
        usage()
    else:
      usage()

  do_remove_comment_from_code()


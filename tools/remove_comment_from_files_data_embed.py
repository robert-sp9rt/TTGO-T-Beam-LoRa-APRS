
import remove_comment_from_code

assets_list = {
    'data_embed/index.html.tmp': 'data_embed/index.html.tmp2',
    'data_embed/js.js': 'data_embed/js.js.tmp',
    'data_embed/style.css': 'data_embed/style.css.tmp'
}


for src_file_name, out_file_name in assets_list.items():
    file_in = open(src_file_name, 'r')
    file_out = open(out_file_name, 'w')
    try:
      if src_file_name.find(".js") > -1:
        remove_comment_from_code.set_one_line_comment_chars("//")
      else:
        remove_comment_from_code.set_one_line_comment_chars([])
      remove_comment_from_code.do_remove_comment_from_code(file_in, file_out)
    except:
      file_out.write(file_in.read())

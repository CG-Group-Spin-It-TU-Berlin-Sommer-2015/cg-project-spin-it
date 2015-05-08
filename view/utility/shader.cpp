int initialize()
{
    ifstream file;
    file.open("shader.vs", "r");
    char *code = "";
    while (!feof(file)) {
        code = code + file >> s;
    }
    int vertex_shader = glCreateShader(GL_VERTEX_SHADER);
    glShaderSource(vertex_shader, code);
    glCompileShader(vertex_shader);
    Log.log(glGetShaderInfoLog(vertex_shader));

    code = "";
    File *file = fopen("shader.fs", "r");
    String code = "";
    while (file >> s) {
        code = code + s;
    }
    int fragment_shader = glCreateShader(GL_FRAGMENT_SHADER);
    glShaderSource(fragment_shader, code);
    glCompileShader(fragment_shader);
    Log.log(glGetShaderInfoLog(fragment_shader));

    int program = glCreateProgram();
    glAttachShader(program, vertex_shader);
    glAttachShader(program, fragment_shader);
    glLinkProgram(program);
    Log.log(glGetProgramInfoLog(program));
    glUseProgram(program);

    return program;
}

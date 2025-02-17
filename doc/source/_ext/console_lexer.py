from pygments.lexer import Lexer, RegexLexer, do_insertions, bygroups, \
    include, default, this, using, words, line_re
from pygments.token import Punctuation, Whitespace, \
    Text, Comment, Operator, Keyword, Name, String, Number, Generic
from pygments.util import shebang_matches


def setup(app):
    app.add_lexer('shell', BashLexerExtended)
    app.add_lexer('console', BashLexerExtended)


class BashLexerExtended(RegexLexer):
    """
    Lexer for (ba|k|z|)sh shell scripts.
    """

    name = 'Bash'
    aliases = ['bash', 'sh', 'ksh', 'zsh', 'shell', 'openrc']
    filenames = ['*.sh', '*.ksh', '*.bash', '*.ebuild', '*.eclass',
                 '*.exheres-0', '*.exlib', '*.zsh',
                 '.bashrc', 'bashrc', '.bash_*', 'bash_*', 'zshrc', '.zshrc',
                 '.kshrc', 'kshrc',
                 'PKGBUILD']
    mimetypes = ['application/x-sh', 'application/x-shellscript', 'text/x-shellscript']
    url = 'https://en.wikipedia.org/wiki/Unix_shell'
    version_added = '0.6'

    tokens = {
        'root': [
            include('basic'),
            (r'`', String.Backtick, 'backticks'),
            include('data'),
            include('interp'),
        ],
        'interp': [
            (r'\$\(\(', Keyword, 'math'),
            (r'\$\(', Keyword, 'paren'),
            (r'\$\{#?', String.Interpol, 'curly'),
            (r'\$[a-zA-Z_]\w*', Name.Variable),  # user variable
            (r'\$(?:\d+|[#$?!_*@-])', Name.Variable),      # builtin
            (r'\$', Text),
        ],
        'basic': [
            (r'\b(if|fi|else|while|in|do|done|for|then|return|function|case|'
             r'select|break|continue|until|esac|elif)(\s*)\b',
             bygroups(Keyword, Whitespace)),
            (r'\b(alias|bg|bind|builtin|caller|cd|command|compgen|'
             r'complete|declare|dirs|disown|echo|enable|eval|exec|exit|'
             r'export|false|fc|fg|getopts|hash|help|history|jobs|kill|let|'
             r'local|logout|popd|printf|pushd|pwd|read|readonly|set|shift|'
             r'shopt|source|suspend|test|time|times|trap|true|type|typeset|'
             r'ulimit|umask|unalias|unset|wait|'
             # Custom added keywords
             r'sudo|apt|apt-get|apt-cache|mkdir|rosdep|git|roslaunch|pip|vcs|catkin_make|catkin|'
             r'python|make'
             # End of custom added keywords
             r')(?=[\s)`])',
             Name.Builtin),
            (r'\A#!.+\n', Comment.Hashbang),
            (r'#.*\n', Comment.Single),
            (r'\\[\w\W]', String.Escape),
            (r'(\b\w+)(\s*)(\+?=)', bygroups(Name.Variable, Whitespace, Operator)),
            (r'[\[\]{}()=]', Operator),
            (r'<<<', Operator),  # here-string
            (r'<<-?\s*(\'?)\\?(\w+)[\w\W]+?\2', String),
            (r'&&|\|\|', Operator),
        ],
        'data': [
            (r'(?s)\$?"(\\.|[^"\\$])*"', String.Double),
            (r'"', String.Double, 'string'),
            (r"(?s)\$'(\\\\|\\[0-7]+|\\.|[^'\\])*'", String.Single),
            (r"(?s)'.*?'", String.Single),
            (r';', Punctuation),
            (r'&', Punctuation),
            (r'\|', Punctuation),
            (r'\s+', Whitespace),
            (r'\d+\b', Number),
            (r'[^=\s\[\]{}()$"\'`\\<&|;]+', Text),
            (r'<', Text),
        ],
        'string': [
            (r'"', String.Double, '#pop'),
            (r'(?s)(\\\\|\\[0-7]+|\\.|[^"\\$])+', String.Double),
            include('interp'),
        ],
        'curly': [
            (r'\}', String.Interpol, '#pop'),
            (r':-', Keyword),
            (r'\w+', Name.Variable),
            (r'[^}:"\'`$\\]+', Punctuation),
            (r':', Punctuation),
            include('root'),
        ],
        'paren': [
            (r'\)', Keyword, '#pop'),
            include('root'),
        ],
        'math': [
            (r'\)\)', Keyword, '#pop'),
            (r'\*\*|\|\||<<|>>|[-+*/%^|&<>]', Operator),
            (r'\d+#[\da-zA-Z]+', Number),
            (r'\d+#(?! )', Number),
            (r'0[xX][\da-fA-F]+', Number),
            (r'\d+', Number),
            (r'[a-zA-Z_]\w*', Name.Variable),  # user variable
            include('root'),
        ],
        'backticks': [
            (r'`', String.Backtick, '#pop'),
            include('root'),
        ],
    }

    def analyse_text(text):
        if shebang_matches(text, r'(ba|z|)sh'):
            return 1
        if text.startswith('$ '):
            return 0.2
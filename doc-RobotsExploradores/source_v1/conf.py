# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys

# Directorio del archivo conf.py
current_dir = os.path.dirname(os.path.abspath(__file__))

# Agrega los directorios 'src' al PYTHONPATH para encontrar módulos
sys.path.insert(0, os.path.abspath(os.path.join(current_dir, '../../squad_main/src')))
sys.path.insert(0, os.path.abspath(os.path.join(current_dir, '../../squad_exploracion/src')))
sys.path.insert(0, os.path.abspath(os.path.join(current_dir, '../../squad_navegacion/src')))
sys.path.insert(0, os.path.abspath(os.path.join(current_dir, '../../squad_planificacion/src')))
sys.path.insert(0, os.path.abspath(os.path.join(current_dir, '../../squad_simulacion/src')))



# -- Project information -----------------------------------------------------
project = 'Robots Exploradores - ROS 1'
copyright = '2024, Abel Belhaki Rivas'
author = 'Abel Belhaki Rivas'
release = '2.0'

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',              # Generación automática de documentación de docstrings
    'sphinx.ext.napoleon',             # Soporte para Google y NumPy docstrings
    'sphinx_autodoc_typehints',        # Documentación de anotaciones de tipo
    'sphinx.ext.graphviz',             # Soporte para diagramas Graphviz
    'sphinx.ext.viewcode',             # Añade enlaces al código fuente
    'sphinx.ext.todo',                 # Soporte para TODOs en la documentación
    'sphinx.ext.githubpages',          # Soporte para GitHub Pages
    'sphinx.ext.intersphinx',          # Enlazar con documentación de otros proyectos
    'sphinx.ext.mathjax',              # Renderizado de expresiones matemáticas
]

html_favicon = '_static/favicon.ico'

# Configuración de Napoleon para soportar Google/NumPy docstrings
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_private_with_doc = False
napoleon_include_special_with_doc = True

# Estilo de resaltado de código
pygments_style = 'sphinx'
highlight_language = 'python'

# Rutas de plantillas
templates_path = ['_templates']
exclude_patterns = []

# Configuración del idioma
language = 'es'

# Mock de módulos no disponibles en el entorno actual (por ejemplo, módulos ROS)
autodoc_mock_imports = [
    'rospy',
    'std_msgs',
    'geometry_msgs',
    'sensor_msgs',
    'actionlib',
    'smach',
    'smach_ros',
    'cv_bridge',
    'cv2',
    'PIL',
]

# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_logo = "_static/logoUA.png"  # Asegúrate de añadir un logo en esta ruta
html_favicon = "_static/faviconUA.ico"  # Asegúrate de añadir un favicon
html_theme_options = {
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'titles_only': False
}

# -- Options for LaTeX output ------------------------------------------------
latex_elements = {
    'papersize': 'a4paper',
    'pointsize': '10pt',
    'preamble': r'''
        \usepackage{charter} % Cambia el tipo de letra
        \usepackage{amsmath} % Paquete para ecuaciones
        \setcounter{tocdepth}{2} % Ajusta la profundidad del índice
        \renewcommand{\familydefault}{\sfdefault} % Usa una fuente sans-serif por defecto
    ''',
    'figure_align': 'H',  # Asegura que las figuras se alineen correctamente
}

latex_documents = [
    ('index', 'RobotsExploradores-ROS1.tex', 
     'Robots Exploradores - ROS 1',
     'Abel Belhaki Rivas', 'manual'),
]

# -- Extensiones adicionales -------------------------------------------------
# Configuración de intersphinx para enlazar a otras documentaciones
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    # 'ros': ('http://wiki.ros.org', None),
}

# -- Configuración para "TODOs" ----------------------------------------------
todo_include_todos = True  # Incluye los TODOs en la salida

# -- Configuración para MathJax ----------------------------------------------
mathjax_config = {
    'TeX': {
        'Macros': {
            'RR': r'\mathbb{R}',
            'vec': [r'\mathbf{#1}', 1],
        }
    }
}

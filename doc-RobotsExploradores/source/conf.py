# Configuration file for the Sphinx documentation builder.
#
# Para la lista completa de opciones y detalles, consulta:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

import os
import sys

##############################################################################
# Ajuste de ruta para que Sphinx localice el código fuente de los paquetes
##############################################################################
current_dir = os.path.dirname(os.path.abspath(__file__))

# Agrega los directorios 'src' al PYTHONPATH para encontrar módulos
sys.path.insert(0, os.path.abspath(os.path.join(current_dir, '../../squad_main/src')))
sys.path.insert(0, os.path.abspath(os.path.join(current_dir, '../../squad_exploracion/src')))
sys.path.insert(0, os.path.abspath(os.path.join(current_dir, '../../squad_navegacion/src')))
sys.path.insert(0, os.path.abspath(os.path.join(current_dir, '../../squad_planificacion/src')))
sys.path.insert(0, os.path.abspath(os.path.join(current_dir, '../../squad_simulacion/src')))

##############################################################################
# -- Project information -----------------------------------------------------
##############################################################################
project = 'Robots Exploradores - ROS 1'
copyright = '2024, Abel Belhaki Rivas'
author = 'Abel Belhaki Rivas'
release = '3.1'  # versión del proyecto

##############################################################################
# -- General configuration ---------------------------------------------------
##############################################################################
# Aquí activamos todas las extensiones que pueden ser útiles
extensions = [
    # ----------------- Autodocumentación y tipados -----------------
    'sphinx.ext.autodoc',                # Generación automática a partir de docstrings
    'sphinx.ext.napoleon',               # Soporte para Google/NumPy docstrings
    'sphinx_autodoc_typehints',          # Muestra anotaciones de tipo en la doc

    # ----------------- Referencias cruzadas e intersphinx -----------------
    'sphinx.ext.intersphinx',            # Enlazar con otras documentaciones (p.ej. Python)
    
    # ----------------- Diagramas y visualización de código -----------------
    'sphinx.ext.viewcode',               # Enlaces al código fuente
    'sphinx.ext.graphviz',               # Soporte para diagramas con Graphviz

    # ----------------- Diagramas PlantUML (UML, etc.) -----------------
    'sphinxcontrib.plantuml',            # Necesitas 'pip install sphinxcontrib-plantuml'
    # Recuerda también instalar PlantUML y Java. Por ejemplo:
    #   sudo apt-get install default-jre
    #   wget https://downloads.sourceforge.net/project/plantuml/plantuml.jar
    #   (o instalar plantuml desde repositorios)

    # ----------------- Texto en Markdown con MyST -----------------
    'myst_parser',                       # pip install myst-parser
    # Con esta extensión puedes mezclar RST con archivos .md, y usar directivas Sphinx en Markdown

    # ----------------- Cobertura de la documentación -----------------
    'sphinx.ext.coverage',               # Genera informes de qué está (o no) documentado

    # ----------------- Pruebas de los ejemplos en doc -----------------
    'sphinx.ext.doctest',               # Convierte ejemplos de doc en tests (make doctest)

    # ----------------- Manejo de bibliografía -----------------
    #'sphinxcontrib.bibtex',             # pip install sphinxcontrib-bibtex
    # Podrás usar :cite: y un .bib para referencias

    # ----------------- Manejo de TODOs -----------------
    'sphinx.ext.todo',                  # Directiva .. todo:: y mostrarlos en la doc

    # ----------------- Subir a GitHub Pages -----------------
    'sphinx.ext.githubpages',           # Para publicar fácilmente en GitHub Pages

    # ----------------- Autoetiquetado de secciones -----------------
    'sphinx.ext.autosectionlabel',      # Permite referenciar secciones por su título
    # Ojo con colisiones de títulos en distintos .rst
]

##############################################################################
# Config Napoleon (docstrings)
##############################################################################
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_private_with_doc = False
napoleon_include_special_with_doc = True

##############################################################################
# Opciones para autodoc
##############################################################################
autodoc_member_order = 'bysource'  # Muestra miembros en el orden del código fuente
autoclass_content = 'both'         # Incluye docstring de la clase y de __init__
autodoc_typehints = 'description'  # Anotaciones de tipo dentro de la descripción

##############################################################################
# Estilo y resaltado
##############################################################################
pygments_style = 'sphinx'
highlight_language = 'python'

##############################################################################
# Rutas de plantillas y patrones de exclusión
##############################################################################
templates_path = ['_templates']
exclude_patterns = []

##############################################################################
# Idioma de la documentación
##############################################################################
language = 'es'

##############################################################################
# Módulos a "mockear" (porque no están disponibles en tiempo de build)
##############################################################################
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
    # Añade aquí cualquier otro módulo ROS o librería no disponible
]

##############################################################################
# -- Options for HTML output -------------------------------------------------
##############################################################################
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']

html_logo = "_static/logo.jpeg"      
html_favicon = "_static/favicon.ico"  

html_theme_options = {
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 4,
    'titles_only': False
}

##############################################################################
# -- Opciones de LaTeX/PDF ---------------------------------------------------
##############################################################################
latex_elements = {
    'papersize': 'a4paper',
    'pointsize': '10pt',
    'preamble': r'''
        \usepackage{charter}     % Cambia el tipo de letra (opcional)
        \usepackage{amsmath}     % Para ecuaciones matemáticas
        \setcounter{tocdepth}{2} % Ajusta profundidad del índice
        \renewcommand{\familydefault}{\sfdefault} % Usa sans-serif por defecto
    ''',
    'figure_align': 'H',  # Asegura que las figuras se alineen correctamente
}

# Si prefieres XeLaTeX, habilita esto:
# latex_engine = 'xelatex'

latex_documents = [
    ('index', 'RobotsExploradores-ROS1.tex',
     'Robots Exploradores - ROS 1',
     'Abel Belhaki Rivas', 'manual'),
]

##############################################################################
# -- Otras configuraciones ---------------------------------------------------
##############################################################################
# Intersphinx: para enlazar a la doc de Python u otros
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    #'ros': ('http://wiki.ros.org', None),  
}

# Habilita la directiva .. todo:: en la salida
todo_include_todos = True

# Para diagramas PlantUML, define dónde está plantuml.jar (opcional).
# plantuml = 'java -jar /ruta/a/plantuml.jar'
# O si está en el PATH, bastaría con 'plantuml = "plantuml"' o similar.

##############################################################################
# -- Configuración para autosectionlabel -------------------------------------
##############################################################################
# Añade el prefijo del documento a las etiquetas de sección para evitar colisiones
autosectionlabel_prefix_document = True

##############################################################################
# -- Configuración para MathJax ----------------------------------------------
##############################################################################
mathjax_config = {
    'TeX': {
        'Macros': {
            'RR': r'\mathbb{R}',
            'vec': [r'\mathbf{#1}', 1],
        }
    }
}

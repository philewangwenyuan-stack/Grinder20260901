"""Use the packaged DOCX renderer with PDFium rasterization on Windows."""
import importlib.util
from pathlib import Path
import sys
import pypdfium2 as pdfium

path = Path('C:/Users/phil/.codex/plugins/cache/openai-primary-runtime/documents/26.1011.11113/skills/documents/render_docx.py')
spec = importlib.util.spec_from_file_location('docx_renderer', path)
renderer = importlib.util.module_from_spec(spec)
spec.loader.exec_module(renderer)

def render_pages(pdf_path, dpi, output_folder, **kwargs):
    document = pdfium.PdfDocument(pdf_path)
    files = []
    for number in range(len(document)):
        destination = Path(output_folder) / ('pdfium-%d.png' % (number + 1))
        document[number].render(scale=dpi/72).to_pil().save(destination)
        files.append(str(destination))
    return files

renderer.convert_from_path = render_pages
renderer.main()

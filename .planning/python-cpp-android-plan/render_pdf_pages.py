from pathlib import Path
import pypdfium2 as pdfium

root = Path(r"E:\CODE\C++\Grinder\.planning\python-cpp-android-plan\rendered")
pdf = pdfium.PdfDocument(root / "preview5.pdf")
for index in range(len(pdf)):
    page = pdf[index]
    image = page.render(scale=1.6).to_pil()
    image.save(root / f"qa5-page-{index + 1:02d}.png")
print(len(pdf))

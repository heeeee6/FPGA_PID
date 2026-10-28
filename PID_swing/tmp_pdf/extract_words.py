import sys, pdfplumber

pdf_path = r"C:\Users\ASUS\Desktop\SmartZynq_SL_Schematic_V1d3B_20260415.pdf"
pages = [9, 11] if len(sys.argv) < 2 else [int(x) for x in sys.argv[1:]]

with pdfplumber.open(pdf_path) as pdf:
    for pno in pages:
        page = pdf.pages[pno - 1]
        print(f"===== PAGE {pno} (rot={page.rotation}) =====")
        words = page.extract_words(use_text_flow=False, keep_blank_chars=False, extra_attrs=["size"])
        # sort top-to-bottom, then left-to-right
        words.sort(key=lambda w: (round(w["top"] / 8), round(w["x0"] / 4)))
        for w in words:
            print(f'{w["top"]:8.1f} {w["x0"]:8.1f} {w["text"]}')

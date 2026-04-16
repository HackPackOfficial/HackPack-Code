/**
 * BMP Editor - Image converter for String Plotter
 */

import { FC, useState, ChangeEvent, useRef, useEffect, MouseEvent } from 'react';
import Image from 'next/image';
import { FaImage } from 'react-icons/fa';

type TabMode = 'upload' | 'draw';

// Drawing Canvas Component
interface DrawingCanvasProps {
  width: number;
  height: number;
  onDrawingChange: (dataUrl: string) => void;
}

const DrawingCanvas: FC<DrawingCanvasProps> = ({ width, height, onDrawingChange }) => {
  const canvasRef = useRef<HTMLCanvasElement>(null);
  const [isDrawing, setIsDrawing] = useState(false);
  const [canvasData, setCanvasData] = useState<string | null>(null);
  const [brushColor, setBrushColor] = useState('#000000');
  const [brushSize, setBrushSize] = useState(2);

  useEffect(() => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.strokeStyle = brushColor;
    ctx.lineWidth = brushSize;
    ctx.lineCap = 'round';
    ctx.lineJoin = 'round';

    if (canvasData) {
      const img = new window.Image();
      img.onload = () => {
        ctx.fillStyle = 'white';
        ctx.fillRect(0, 0, width, height);
        ctx.drawImage(img, 0, 0, width, height);
        onDrawingChange(canvas.toDataURL());
      };
      img.src = canvasData;
    } else {
      ctx.fillStyle = 'white';
      ctx.fillRect(0, 0, width, height);
      onDrawingChange(canvas.toDataURL());
    }
  }, [width, height, onDrawingChange, canvasData, brushColor, brushSize]);

  const startDrawing = (e: MouseEvent<HTMLCanvasElement>) => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    setIsDrawing(true);
    ctx.beginPath();
    ctx.moveTo(x, y);
  };

  const draw = (e: MouseEvent<HTMLCanvasElement>) => {
    if (!isDrawing) return;

    const canvas = canvasRef.current;
    if (!canvas) return;

    const rect = canvas.getBoundingClientRect();
    const x = e.clientX - rect.left;
    const y = e.clientY - rect.top;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.lineTo(x, y);
    ctx.stroke();
  };

  const stopDrawing = () => {
    if (!isDrawing) return;

    setIsDrawing(false);
    const canvas = canvasRef.current;
    if (canvas) {
      const dataUrl = canvas.toDataURL();
      setCanvasData(dataUrl);
      onDrawingChange(dataUrl);
    }
  };

  const clearCanvas = () => {
    const canvas = canvasRef.current;
    if (!canvas) return;

    const ctx = canvas.getContext('2d');
    if (!ctx) return;

    ctx.fillStyle = 'white';
    ctx.fillRect(0, 0, width, height);
    const dataUrl = canvas.toDataURL();
    setCanvasData(dataUrl);
    onDrawingChange(dataUrl);
  };

  return (
    <div className='space-y-2'>
      <div className='my-10 flex items-center gap-10'>
        <div className='flex items-center gap-4'>
          <label htmlFor='colorPicker' className='text-sm font-medium text-white'>
            Color:
          </label>
          <input
            id='colorPicker'
            type='color'
            value={brushColor}
            onChange={(e) => setBrushColor(e.target.value)}
            className='h-20 w-20 cursor-pointer'
            style={{ backgroundColor: brushColor }}
          />
        </div>
        <div className='flex items-center gap-2'>
          <label htmlFor='brushSize' className='w-75 text-sm font-medium text-white'>
            Brush: {brushSize}px
          </label>
          <input id='brushSize' type='range' min='1' max='20' value={brushSize} onChange={(e) => setBrushSize(Number(e.target.value))} className='w-80' />
        </div>
      </div>
      <canvas
        ref={canvasRef}
        width={width}
        height={height}
        onMouseDown={startDrawing}
        onMouseMove={draw}
        onMouseUp={stopDrawing}
        onMouseLeave={stopDrawing}
        className='mx-auto cursor-crosshair border border-gray bg-white'
      />
      <button type='button' onClick={clearCanvas} className='hover:bg-gray-600 rounded bg-gray px-3 py-1 text-sm text-white'>
        Clear
      </button>
    </div>
  );
};

// Scaling Controls Component
interface ScalingControlsProps {
  scaleX: number;
  scaleY: number;
  onScaleXChange: (value: number) => void;
  onScaleYChange: (value: number) => void;
  onReset: () => void;
}

const ScalingControls: FC<ScalingControlsProps> = ({ scaleX, scaleY, onScaleXChange, onScaleYChange, onReset }) => {
  const [preserveAspectRatio, setPreserveAspectRatio] = useState(false);
  const aspectRatio = scaleX / scaleY;

  const handleScaleXChange = (value: number) => {
    onScaleXChange(value);
    if (preserveAspectRatio) {
      onScaleYChange(Math.round(value / aspectRatio));
    }
  };

  const handleScaleYChange = (value: number) => {
    onScaleYChange(value);
    if (preserveAspectRatio) {
      onScaleXChange(Math.round(value * aspectRatio));
    }
  };

  return (
    <div className='space-y-4'>
      <h2 className='py-5'>Sizing</h2>
      <div className='!mb-10 flex items-center gap-4'>
        <label className='flex items-center gap-2 text-sm text-white'>
          <input type='checkbox' checked={preserveAspectRatio} onChange={(e) => setPreserveAspectRatio(e.target.checked)} className='rounded' />
          Preserve aspect ratio
        </label>
      </div>
      <div className='grid grid-cols-[120px_1fr]'>
        <label htmlFor='scaleX' className='mb-2 block text-sm font-medium'>
          Width: {scaleX}px
        </label>
        <input
          id='scaleX'
          type='range'
          min='100'
          max='350'
          value={scaleX}
          onChange={(e) => handleScaleXChange(Number(e.target.value))}
          className='bg-gray-200 h-2 cursor-pointer appearance-none self-center rounded-lg'
        />
      </div>
      <div className='grid grid-cols-[120px_1fr]'>
        <label htmlFor='scaleY' className='mb-2 block text-sm font-medium'>
          Height: {scaleY}px
        </label>
        <input
          id='scaleY'
          type='range'
          min='100'
          max='350'
          value={scaleY}
          onChange={(e) => handleScaleYChange(Number(e.target.value))}
          className='bg-gray-200 h-2 cursor-pointer appearance-none self-center rounded-lg'
        />
      </div>
      <div className='!mb-10'>
        <button type='button' onClick={onReset} className='rounded bg-cl-orange2 px-4 py-5 text-sm text-white hover:bg-cl-orange3'>
          Reset Size
        </button>
      </div>
    </div>
  );
};

// Grayscale Controls Component
interface GrayscaleControlsProps {
  onGrayscaleApply: (dataUrl: string) => void;
  onReset: () => void;
  imageUrl: string;
}

const GrayscaleControls: FC<GrayscaleControlsProps> = ({ onGrayscaleApply, onReset, imageUrl }) => {
  const getGrayscaleValues = (shades: number) => {
    const values = [];
    for (let i = 0; i < shades; i += 1) {
      const value = Math.round((i * 255) / (shades - 1));
      values.push(value);
    }
    return values;
  };

  const applyGrayscale = (shades: number) => {
    if (!imageUrl) return;

    const img = new window.Image();
    img.onload = () => {
      const canvas = document.createElement('canvas');
      canvas.width = img.width;
      canvas.height = img.height;

      const ctx = canvas.getContext('2d');
      if (!ctx) return;

      ctx.drawImage(img, 0, 0);
      const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
      const { data } = imageData;

      for (let i = 0; i < data.length; i += 4) {
        const gray = Math.round(0.299 * data[i] + 0.587 * data[i + 1] + 0.114 * data[i + 2]);
        const quantized = Math.floor(gray / (256 / shades)) * (255 / (shades - 1));

        data[i] = quantized;
        data[i + 1] = quantized;
        data[i + 2] = quantized;
      }

      ctx.putImageData(imageData, 0, 0);
      onGrayscaleApply(canvas.toDataURL());
    };
    img.src = imageUrl;
  };

  return (
    <div className='space-y-2'>
      <h4 className='mb-10 text-sm font-medium text-white'>Grayscale</h4>
      <div className='flex items-center gap-2'>
        <button type='button' onClick={() => applyGrayscale(3)} className='bg-gray-600 rounded px-3 py-1 text-sm text-white hover:bg-cl-gray'>
          <div className='flex gap-1'>
            <span className='mr-5'>3 shades</span>
            {getGrayscaleValues(3).map((value) => (
              <div
                key={value}
                className='h-20 w-20 border border-gray'
                style={{ backgroundColor: `rgb(${value}, ${value}, ${value})` }}
                title={`Gray: ${value}`}
              />
            ))}
          </div>
        </button>
      </div>
      <div className='flex items-center gap-2'>
        <button type='button' onClick={() => applyGrayscale(4)} className='bg-gray-600 rounded px-3 py-1 text-sm text-white hover:bg-cl-gray'>
          <div className='flex gap-1'>
            <span className='mr-5'>4 shades</span>
            {getGrayscaleValues(4).map((value) => (
              <div
                key={value}
                className='h-20 w-20 border border-gray'
                style={{ backgroundColor: `rgb(${value}, ${value}, ${value})` }}
                title={`Gray: ${value}`}
              />
            ))}
          </div>
        </button>
      </div>
      <div className='flex items-center gap-2'>
        <button type='button' onClick={() => applyGrayscale(5)} className='bg-gray-600 rounded px-3 py-1 text-sm text-white hover:bg-cl-gray'>
          <div className='flex gap-1'>
            <span className='mr-5'>5 shades</span>
            {getGrayscaleValues(5).map((value) => (
              <div
                key={value}
                className='h-20 w-20 border border-gray'
                style={{ backgroundColor: `rgb(${value}, ${value}, ${value})` }}
                title={`Gray: ${value}`}
              />
            ))}
          </div>
        </button>
      </div>
      <div className='flex items-center gap-2'>
        <button type='button' onClick={() => applyGrayscale(6)} className='bg-gray-600 rounded px-3 py-1 text-sm text-white hover:bg-cl-gray'>
          <div className='flex gap-1'>
            <span className='mr-5'>6 shades</span>
            {getGrayscaleValues(6).map((value) => (
              <div
                key={value}
                className='h-20 w-20 border border-gray'
                style={{ backgroundColor: `rgb(${value}, ${value}, ${value})` }}
                title={`Gray: ${value}`}
              />
            ))}
          </div>
        </button>
      </div>
      <button type='button' onClick={onReset} className='!my-10 rounded bg-cl-orange2 px-4 py-5 text-sm text-white hover:bg-cl-orange1'>
        Reset to original
      </button>
    </div>
  );
};

// Main BMP Editor Component
interface BMPEditorProps {
  onImageConverted?: (bmpData: string) => void;
}

const PlotterBMPEditor: FC<BMPEditorProps> = ({ onImageConverted }) => {
  const [error, setError] = useState<string>('');
  const [previewUrl, setPreviewUrl] = useState<string>('');
  const [originalUrl, setOriginalUrl] = useState<string>('');
  const [scaleX, setScaleX] = useState<number>(250);
  const [scaleY, setScaleY] = useState<number>(250);
  const [originalScaleX, setOriginalScaleX] = useState<number>(250);
  const [originalScaleY, setOriginalScaleY] = useState<number>(250);
  const [activeTab, setActiveTab] = useState<TabMode>('upload');

  const createBMP = (imageData: ImageData): Uint8Array => {
    const { width, height } = imageData;
    const bmpSize = 54 + width * height * 3;
    const data = new Uint8Array(bmpSize);

    // BMP File Header (14 bytes)
    data[0] = 0x42;
    data[1] = 0x4d;
    data[2] = bmpSize % 256;
    data[3] = Math.floor(bmpSize / 256) % 256;
    data[4] = Math.floor(bmpSize / 65536) % 256;
    data[5] = Math.floor(bmpSize / 16777216) % 256;
    data[10] = 54;

    // BMP Info Header (40 bytes)
    data[14] = 40;
    data[18] = width % 256;
    data[19] = Math.floor(width / 256) % 256;
    data[22] = height % 256;
    data[23] = Math.floor(height / 256) % 256;
    data[26] = 1;
    data[28] = 24;

    // Pixel data (BGR format, bottom-to-top)
    let pos = 54;
    const padding = (4 - ((width * 3) % 4)) % 4;

    for (let y = height - 1; y >= 0; y -= 1) {
      for (let x = 0; x < width; x += 1) {
        const pixelIndex = (y * width + x) * 4;
        data[pos] = imageData.data[pixelIndex + 2];
        pos += 1;
        data[pos] = imageData.data[pixelIndex + 1];
        pos += 1;
        data[pos] = imageData.data[pixelIndex];
        pos += 1;
      }
      pos += padding;
    }

    return data;
  };

  const convertToBmp = () => {
    if (!previewUrl) return;

    const img = new window.Image();
    img.onload = () => {
      const canvas = document.createElement('canvas');
      canvas.width = scaleX;
      canvas.height = scaleY;

      const ctx = canvas.getContext('2d');
      if (!ctx) {
        setError('Failed to get canvas context');
        return;
      }

      ctx.fillStyle = 'white';
      ctx.fillRect(0, 0, scaleX, scaleY);
      ctx.drawImage(img, 0, 0, scaleX, scaleY);

      const imageData = ctx.getImageData(0, 0, canvas.width, canvas.height);
      const bmpData = createBMP(imageData);

      // @ts-ignore
      const blob = new Blob([bmpData], { type: 'image/bmp' });
      const url = URL.createObjectURL(blob);
      const timestamp = Date.now();
      const filename = `string_plotter_image_${timestamp}.bmp`;

      const link = document.createElement('a');
      link.href = url;
      link.download = filename;
      document.body.appendChild(link);
      link.click();
      document.body.removeChild(link);
      URL.revokeObjectURL(url);

      if (onImageConverted) {
        const base64String = btoa(String.fromCharCode.apply(null, Array.from(bmpData)));
        onImageConverted(`data:image/bmp;base64,${base64String}`);
      }
    };
    img.src = previewUrl;
  };

  const handleDrawingChange = (dataUrl: string) => {
    setPreviewUrl(dataUrl);
    setOriginalUrl(dataUrl);
    setOriginalScaleX(scaleX);
    setOriginalScaleY(scaleY);
  };

  const handleGrayscaleApply = (dataUrl: string) => {
    setPreviewUrl(dataUrl);
  };

  const handleGrayscaleReset = () => {
    if (originalUrl) {
      setPreviewUrl(originalUrl);
    }
  };

  const handleScaleReset = () => {
    setScaleX(originalScaleX);
    setScaleY(originalScaleY);
  };

  const handleFileChange = (event: ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (file) {
      if (file.type.startsWith('image/')) {
        setError('');
        const reader = new FileReader();
        reader.onload = (e) => {
          const dataUrl = e.target?.result as string;

          const img = new window.Image();
          img.onload = () => {
            const aspectRatio = img.width / img.height;
            let newWidth = 250;
            let newHeight = 250;

            if (aspectRatio > 1) {
              newHeight = Math.round(newWidth / aspectRatio);
            } else {
              newWidth = Math.round(newHeight * aspectRatio);
            }

            setScaleX(newWidth);
            setScaleY(newHeight);
            setOriginalScaleX(newWidth);
            setOriginalScaleY(newHeight);
            setPreviewUrl(dataUrl);
            setOriginalUrl(dataUrl);
          };
          img.src = dataUrl;
        };
        reader.readAsDataURL(file);
      } else {
        setError('Please select an image file');
      }
    }
  };

  return (
    <div className='py-5 text-white'>
      {/* Tabs */}
      <div className='flex border-b border-gray'>
        <button
          type='button'
          onClick={() => setActiveTab('upload')}
          className={`px-4 py-2 font-medium ${activeTab === 'upload' ? 'text-blue border-b-2 border-cl-orange1' : 'text-white hover:text-gray'}`}
        >
          Upload Image
        </button>
        <button
          type='button'
          onClick={() => setActiveTab('draw')}
          className={`px-4 py-2 font-medium ${activeTab === 'draw' ? 'text-blue border-b-2 border-cl-orange1' : 'text-white hover:text-gray'}`}
        >
          Drawing Canvas
        </button>
      </div>

      {/* Tab Content */}
      <div className='space-y-4'>
        {activeTab === 'upload' && (
          <div className='w-full'>
            <input type='file' accept='image/*' onChange={handleFileChange} className='w-full rounded py-5' />
            {error && <p className='text-red-500 mt-1'>{error}</p>}
          </div>
        )}

        {activeTab === 'draw' && <DrawingCanvas width={scaleX} height={scaleY} onDrawingChange={handleDrawingChange} />}

        {/* Preview - hidden in drawing mode */}
        {activeTab !== 'draw' && (
          <div className='rounded border border-gray p-4'>
            <h3 className='mb-2 text-sm font-medium'>Preview:</h3>
            {previewUrl ? (
              <Image
                key={`${scaleX}-${scaleY}`}
                src={previewUrl}
                alt='Preview'
                width={scaleX}
                height={scaleY}
                className='mx-auto border bg-white'
                style={{ height: scaleY }}
                unoptimized
              />
            ) : (
              <div
                className='mx-auto flex items-center justify-center border border-dashed border-gray bg-darkGray !text-gray'
                style={{ width: scaleX, height: scaleY }}
              >
                <FaImage size={48} />
              </div>
            )}
          </div>
        )}

        <ScalingControls scaleX={scaleX} scaleY={scaleY} onScaleXChange={setScaleX} onScaleYChange={setScaleY} onReset={handleScaleReset} />

        {previewUrl && activeTab !== 'draw' && (
          <>
            <div className='border border-gray' />
            <GrayscaleControls imageUrl={originalUrl || previewUrl} onGrayscaleApply={handleGrayscaleApply} onReset={handleGrayscaleReset} />
            <div className='border border-gray' />
          </>
        )}

        <button
          type='button'
          onClick={convertToBmp}
          disabled={!previewUrl}
          className='!my-10 rounded bg-cl-orange2 px-10 py-8 text-white hover:bg-cl-orange1 disabled:cursor-not-allowed disabled:bg-gray'
        >
          Convert to BMP
        </button>
      </div>
    </div>
  );
};

export default PlotterBMPEditor;

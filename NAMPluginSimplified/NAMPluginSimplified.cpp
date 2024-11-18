#include "NAMPluginSimplified.h"
#include "IPlug_include_in_plug_src.h"
#include "IControls.h"
#include "ir.h"
#include "model.h"

#include "architecture.hpp"

const double kDCBlockerFrequency = 5.0;
NAMPluginSimplified::NAMPluginSimplified(const InstanceInfo& info)
    : Plugin(info, MakeConfig(kNumParams, kNumPresets))
{
    nam::activations::Activation::enable_fast_tanh();

    GetParam(kInputLevel)->InitGain("Input", 0.0, -20.0, 20.0, 0.1);
    GetParam(kOutputLevel)->InitGain("Output", -6.0, -40.0, 40.0, 0.1);
    GetParam(kModelToggle)->InitBool("ModelToggle", true);
    GetParam(kIRToggle)->InitBool("IRToggle", true);
    GetParam(kOutNormToggle)->InitBool("OutNormToggle", true);

    //#if IPLUG_EDITOR // http://bit.ly/2S64BDd
    mMakeGraphicsFunc = [&]() {
        return MakeGraphics(*this, PLUG_WIDTH, PLUG_HEIGHT, PLUG_FPS, GetScaleForScreen(PLUG_WIDTH, PLUG_HEIGHT));
        };

    mLayoutFunc = [&](IGraphics* pGraphics) {
        pGraphics->AttachCornerResizer(EUIResizerMode::Scale, false);
        //pGraphics->AttachPanelBackground(COLOR_GRAY);
        pGraphics->LoadFont("Roboto-Regular", ROBOTO_FN);
        pGraphics->LoadBitmap(BACKGROUND_FN);

        const IRECT b = pGraphics->GetBounds();
        const auto mainArea = b.GetPadded(-20);
        const auto contentArea = mainArea.GetPadded(-10);

        // Areas for knobs
        const auto knobsPad = 20.0f;
        const auto knobHeight = 120.f;
        const auto singleKnobPad = -2.0f;
        const auto knobsArea =
            contentArea.GetFromTop(knobHeight).GetReducedFromLeft(knobsPad).GetReducedFromRight(knobsPad);
        const auto inputKnobArea = knobsArea.GetGridCell(0, 0, 1, numKnobs).GetPadded(-singleKnobPad);
        const auto outputKnobArea = knobsArea.GetGridCell(0, 1, 1, numKnobs).GetPadded(-singleKnobPad);

        // Areas for toggles
        const auto ngToggleArea =
            contentArea.GetFromBottom(knobHeight).GetReducedFromLeft(knobsPad).GetReducedFromRight(knobsPad);
        const auto modelToggleArea = ngToggleArea.GetGridCell(1, 0, 3, 3).GetScaledAboutCentre(0.9);
        const auto IRToggleArea = ngToggleArea.GetGridCell(1, 1, 3, 3).GetScaledAboutCentre(0.9);
        const auto outNormToggleArea = ngToggleArea.GetGridCell(1, 2, 3, 3).GetScaledAboutCentre(0.9);

        pGraphics->AttachBackground(BACKGROUND_FN);
        pGraphics->AttachControl(new IVKnobControl(inputKnobArea, kInputLevel));
        pGraphics->AttachControl(new IVKnobControl(outputKnobArea, kOutputLevel));
        pGraphics->AttachControl(new IVSlideSwitchControl(modelToggleArea, kModelToggle, "Enable Model"));
        pGraphics->AttachControl(new IVSlideSwitchControl(IRToggleArea, kIRToggle, "Enable IR"));
        pGraphics->AttachControl(new IVSlideSwitchControl(outNormToggleArea, kOutNormToggle, "Normalize Model"));
        };

    // Load fixed model for now
    WDL_String modelFileName;
    modelFileName.Set(FIXED_MODEL);
    //const std::string msg = _StageModel(modelFileName);
    const std::string msg = _StageModelStream();
    // std::cout << "Loaded model: " << modelFileName.Get() << std::endl;

    // Load fixed IR for now
    WDL_String IRFileName;
    IRFileName.Set(FIXED_IR);
    //const dsp::wav::LoadReturnCode retCode = _StageIR(IRFileName);
    const dsp::wav::LoadReturnCode retCode = _StageIRStream();
    // std::cout << "Loaded IR: " << IRFileName.Get() << std::endl;
  //#endif
}

NAMPluginSimplified::~NAMPluginSimplified()
{
    _DeallocateIOPointers();
}

//#if IPLUG_DSP
void NAMPluginSimplified::ProcessBlock(sample** inputs, sample** outputs, int nFrames)
{
    const size_t numChannelsExternalIn = (size_t)NInChansConnected();
    const size_t numChannelsExternalOut = (size_t)NOutChansConnected();
    const size_t numChannelsInternal = kNumChannelsInternal;
    const size_t numFrames = (size_t)nFrames;
    const double sampleRate = GetSampleRate();

    // Disable floating point denormals
    std::fenv_t fe_state;
    std::feholdexcept(&fe_state);
    disable_denormals();

    _PrepareBuffers(numChannelsInternal, numFrames);

    // Input is collapsed to mono in preparation for the NAM.
    _ProcessInput(inputs, numFrames, numChannelsExternalIn, numChannelsInternal);
    _ApplyDSPStaging();

    if (mModel != nullptr && GetParam(kModelToggle)->Value())
    {
        mModel->process(mInputPointers[0], mOutputPointers[0], nFrames);
        if (GetParam(kOutNormToggle)->Value())
        {
            _NormalizeModelOutput(mOutputPointers, numChannelsInternal, numFrames);
        }
    }
    else
    {
        _FallbackDSP(mInputPointers, mOutputPointers, numChannelsInternal, numFrames);
    }

    sample** irPointers = mOutputPointers;

    if (mIR != nullptr && GetParam(kIRToggle)->Value())
        irPointers = mIR->Process(mOutputPointers, numChannelsInternal, numFrames);

    // restore previous floating point state
    std::feupdateenv(&fe_state);

    // This is where we exit mono for whatever the output requires.
    _ProcessOutput(irPointers, outputs, numFrames, numChannelsInternal, numChannelsExternalOut);

    // * Output of input leveling (inputs -> mInputPointers),
    // * Output of output leveling (mOutputPointers -> outputs)
    //_UpdateMeters(mInputPointers, outputs, numFrames, numChannelsInternal, numChannelsExternalOut);
}
//#endif

// Private methods ============================================================

size_t NAMPluginSimplified::_GetBufferNumChannels() const
{
    // Assumes input=output (no mono->stereo effects)
    return mInputArray.size();
}

size_t NAMPluginSimplified::_GetBufferNumFrames() const
{
    if (_GetBufferNumChannels() == 0)
        return 0;
    return mInputArray[0].size();
}

void NAMPluginSimplified::_AllocateIOPointers(const size_t nChans)
{
    if (mInputPointers != nullptr)
        throw std::runtime_error("Tried to re-allocate mInputPointers without freeing");
    mInputPointers = new sample * [nChans];
    if (mInputPointers == nullptr)
        throw std::runtime_error("Failed to allocate pointer to input buffer!\n");
    if (mOutputPointers != nullptr)
        throw std::runtime_error("Tried to re-allocate mOutputPointers without freeing");
    mOutputPointers = new sample * [nChans];
    if (mOutputPointers == nullptr)
        throw std::runtime_error("Failed to allocate pointer to output buffer!\n");
}

void NAMPluginSimplified::_DeallocateIOPointers()
{
    if (mInputPointers != nullptr)
    {
        delete[] mInputPointers;
        mInputPointers = nullptr;
    }
    if (mInputPointers != nullptr)
        throw std::runtime_error("Failed to deallocate pointer to input buffer!\n");
    if (mOutputPointers != nullptr)
    {
        delete[] mOutputPointers;
        mOutputPointers = nullptr;
    }
    if (mOutputPointers != nullptr)
        throw std::runtime_error("Failed to deallocate pointer to output buffer!\n");
}

void NAMPluginSimplified::_PrepareIOPointers(const size_t numChannels)
{
    _DeallocateIOPointers();
    _AllocateIOPointers(numChannels);
}

void NAMPluginSimplified::_PrepareBuffers(const size_t numChannels, const size_t numFrames)
{
    const bool updateChannels = numChannels != _GetBufferNumChannels();
    const bool updateFrames = updateChannels || (_GetBufferNumFrames() != numFrames);
    //  if (!updateChannels && !updateFrames)  // Could we do this?
    //    return;

    if (updateChannels)
    {
        _PrepareIOPointers(numChannels);
        mInputArray.resize(numChannels);
        mOutputArray.resize(numChannels);
    }
    if (updateFrames)
    {
        for (size_t c = 0; c < mInputArray.size(); c++)
        {
            mInputArray[c].resize(numFrames);
            std::fill(mInputArray[c].begin(), mInputArray[c].end(), 0.0);
        }
        for (size_t c = 0; c < mOutputArray.size(); c++)
        {
            mOutputArray[c].resize(numFrames);
            std::fill(mOutputArray[c].begin(), mOutputArray[c].end(), 0.0);
        }
    }
    // Would these ever get changed by something?
    for (size_t c = 0; c < mInputArray.size(); c++)
        mInputPointers[c] = mInputArray[c].data();
    for (size_t c = 0; c < mOutputArray.size(); c++)
        mOutputPointers[c] = mOutputArray[c].data();
}

void NAMPluginSimplified::_ApplyDSPStaging()
{
    // Move things from staged to live
    if (mStagedModel != nullptr)
    {
        // Move from staged to active DSP
        mModel = std::move(mStagedModel);
        mStagedModel = nullptr;
    }
    if (mStagedIR != nullptr)
    {
        // Move from staged to active IR
        mIR = std::move(mStagedIR);
        mStagedIR = nullptr;
    }
}

std::string NAMPluginSimplified::_StageModel(const WDL_String& modelPath)
{
    try
    {
        auto dspPath = std::filesystem::u8path(modelPath.Get());
        // I added the ResamplingNAM class back. I'm not sure if I'll
        // end up using it with the Daisy Seed, so for now, I assume it will handle it.
        // The old code looks like this:
        // std::unique_ptr<nam::DSP> model = nam::get_dsp(dspPath);
        // mStagedModel = std::move(model);
        // Naturally, both mStagedModel and mModel should be nam::DSP smart pointers as well.
        std::unique_ptr<nam::DSP> model = nam::get_dsp(dspPath);
        std::unique_ptr<ResamplingNAM> temp = std::make_unique<ResamplingNAM>(std::move(model), GetSampleRate());
        temp->Reset(GetSampleRate(), GetBlockSize());
        mStagedModel = std::move(temp);
        mNAMPath = modelPath;
    }
    catch (std::runtime_error& e)
    {
        return e.what();
    }
    return "";
}

std::string NAMPluginSimplified::_StageModelStream()
{
    try
    {
        // I added the ResamplingNAM class back. I'm not sure if I'll
        // end up using it with the Daisy Seed, so for now, I assume it will handle it.
        // The old code looks like this:
        // std::unique_ptr<nam::DSP> model = nam::get_dsp(dspPath);
        // mStagedModel = std::move(model);
        // Naturally, both mStagedModel and mModel should be nam::DSP smart pointers as well.

        std::unique_ptr<nam::DSP> model = nam::get_dsp(modelData, sizeof(modelData));
        std::unique_ptr<ResamplingNAM> temp = std::make_unique<ResamplingNAM>(std::move(model), GetSampleRate());
        temp->Reset(GetSampleRate(), GetBlockSize());
        mStagedModel = std::move(temp);
    }
    catch (std::runtime_error& e)
    {
        return e.what();
    }
    return "";
}

dsp::wav::LoadReturnCode NAMPluginSimplified::_StageIR(const WDL_String& irPath)
{
    // FIXME it'd be better for the path to be "staged" as well. Just in case the
    // path and the model got caught on opposite sides of the fence...
    WDL_String previousIRPath = mIRPath;
    const double sampleRate = GetSampleRate();
    dsp::wav::LoadReturnCode wavState = dsp::wav::LoadReturnCode::ERROR_OTHER;
    try
    {
        auto irPathU8 = std::filesystem::u8path(irPath.Get());
        mStagedIR = std::make_unique<dsp::ImpulseResponse>(irPathU8.string().c_str(), sampleRate);
        wavState = mStagedIR->GetWavState();
    }
    catch (std::runtime_error& e)
    {
        wavState = dsp::wav::LoadReturnCode::ERROR_OTHER;
        // std::cerr << "Caught unhandled exception while attempting to load IR:" << std::endl;
        // std::cerr << e.what() << std::endl;
    }

    if (wavState == dsp::wav::LoadReturnCode::SUCCESS)
    {
        mIRPath = irPath;
        // SendControlMsgFromDelegate(kCtrlTagIRFileBrowser, kMsgTagLoadedIR, mIRPath.GetLength(), mIRPath.Get());
    }
    else
    {
        if (mStagedIR != nullptr)
        {
            mStagedIR = nullptr;
        }
        // mIRPath = previousIRPath;
        // SendControlMsgFromDelegate(kCtrlTagIRFileBrowser, kMsgTagLoadFailed);
    }

    return wavState;
}

dsp::wav::LoadReturnCode NAMPluginSimplified::_StageIRStream()
{
    const double sampleRate = GetSampleRate();
    dsp::wav::LoadReturnCode wavState = dsp::wav::LoadReturnCode::ERROR_OTHER;
    try
    {
        mStagedIR = std::make_unique<dsp::ImpulseResponse>(IRData, sizeof(IRData), sampleRate);
        wavState = mStagedIR->GetWavState();
    }
    catch (std::runtime_error& e)
    {
        wavState = dsp::wav::LoadReturnCode::ERROR_OTHER;
        // std::cerr << "Caught unhandled exception while attempting to load IR:" << std::endl;
        // std::cerr << e.what() << std::endl;
    }

    if (wavState == dsp::wav::LoadReturnCode::SUCCESS)
    {
        // SendControlMsgFromDelegate(kCtrlTagIRFileBrowser, kMsgTagLoadedIR, mIRPath.GetLength(), mIRPath.Get());
    }
    else
    {
        if (mStagedIR != nullptr)
        {
            mStagedIR = nullptr;
        }
        // mIRPath = previousIRPath;
        // SendControlMsgFromDelegate(kCtrlTagIRFileBrowser, kMsgTagLoadFailed);
    }

    return wavState;
}

void NAMPluginSimplified::_FallbackDSP(iplug::sample** inputs, iplug::sample** outputs, const size_t numChannels,
    const size_t numFrames)
{
    for (auto c = 0; c < numChannels; c++)
        for (auto s = 0; s < numFrames; s++)
            mOutputArray[c][s] = mInputArray[c][s];
}

void NAMPluginSimplified::_NormalizeModelOutput(iplug::sample** buffer, const size_t numChannels, const size_t numFrames)
{
    if (!mModel)
        return;
    if (!mModel->HasLoudness())
        return;
    const double loudness = mModel->GetLoudness();
    const double targetLoudness = -18.0;
    const double gain = pow(10.0, (targetLoudness - loudness) / 20.0);
    for (size_t c = 0; c < numChannels; c++)
    {
        for (size_t f = 0; f < numFrames; f++)
        {
            buffer[c][f] *= gain;
        }
    }
}

void NAMPluginSimplified::_ProcessInput(iplug::sample** inputs, const size_t nFrames, const size_t nChansIn,
    const size_t nChansOut)
{
    // We'll assume that the main processing is mono for now. We'll handle dual amps later.
    if (nChansOut != 1)
    {
        // std::stringstream ss;
        // ss << "Expected mono output, but " << nChansOut << " output channels are requested!";
        // throw std::runtime_error(ss.str());
    }

    // On the standalone, we can probably assume that the user has plugged into only one input and they expect it to be
    // carried straight through. Don't apply any division over nCahnsIn because we're just "catching anything out there."
    // However, in a DAW, it's probably something providing stereo, and we want to take the average in order to avoid
    // doubling the loudness.
#ifdef APP_API
    const double gain = pow(10.0, GetParam(kInputLevel)->Value() / 20.0);
#else
    const double gain = pow(10.0, GetParam(kInputLevel)->Value() / 20.0) / (float)nChansIn;
#endif
    // Assume _PrepareBuffers() was already called
    for (size_t c = 0; c < nChansIn; c++)
        for (size_t s = 0; s < nFrames; s++)
            if (c == 0)
                mInputArray[0][s] = gain * inputs[c][s];
            else
                mInputArray[0][s] += gain * inputs[c][s];
}

void NAMPluginSimplified::_ProcessOutput(iplug::sample** inputs, iplug::sample** outputs, const size_t nFrames,
    const size_t nChansIn, const size_t nChansOut)
{
    const double gain = pow(10.0, GetParam(kOutputLevel)->Value() / 20.0);
    // Assume _PrepareBuffers() was already called
    if (nChansIn != 1)
        throw std::runtime_error("Plugin is supposed to process in mono.");
    // Broadcast the internal mono stream to all output channels.
    const size_t cin = 0;
    for (auto cout = 0; cout < nChansOut; cout++)
        for (auto s = 0; s < nFrames; s++)
#ifdef APP_API // Ensure valid output to interface
            outputs[cout][s] = std::clamp(gain * inputs[cin][s], -1.0, 1.0);
#else // In a DAW, other things may come next and should be able to handle large
            // values.
            outputs[cout][s] = gain * inputs[cin][s];
#endif
}
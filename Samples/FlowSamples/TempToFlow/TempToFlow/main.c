/* Copyright (c) Microsoft Corporation. All rights reserved.
   Licensed under the MIT License. */

//This project is based on the HTTPS_Curl_Easy project (original license above and comments below).

// This sample C application for Azure Sphere periodically downloads and outputs the index web page
// at example.com, by using cURL over a secure HTTPS connection.
// It uses the cURL 'easy' API which is a synchronous (blocking) API.
//
// It uses the following Azure Sphere libraries:
// - log (messages shown in Visual Studio's Device Output window during debugging);
// - storage (device storage interaction);
// - curl (URL transfer library).

#include <errno.h>
#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <signal.h>
#include <stdio.h> 

#include <curl/curl.h>

// applibs_versions.h defines the API struct versions to use for applibs APIs.
#include "applibs_versions.h"
#include <applibs/log.h>
#include <applibs/networking.h>
#include <applibs/storage.h>

#include "epoll_timerfd_utilities.h"
#include "i2c.h"
#include "mt3620_avnet_dev.h"
#include "build_options.h"

#include <applibs/log.h>
#include <applibs/i2c.h>

static volatile sig_atomic_t terminationRequired = false;

/// <summary>
///     Signal handler for termination requests. This handler must be async-signal-safe.
/// </summary>
static void TerminationHandler(int signalNumber)
{
    // Don't use Log_Debug here, as it is not guaranteed to be async-signal-safe.
    terminationRequired = true;
}

// Epoll and event handler file descriptors.
static int webpageDownloadTimerFd = -1;
static int epollFd = -1;

/// <summary>
///     Data pointer and size of a block of memory allocated on the heap.
/// </summary>
typedef struct {
    char *data;
    size_t size;
} MemoryBlock;



/// <summary>
///     Logs a cURL error.
/// </summary>
/// <param name="message">The message to print</param>
/// <param name="curlErrCode">The cURL error code to describe</param>
static void LogCurlError(const char *message, int curlErrCode)
{
    Log_Debug(message);
    Log_Debug(" (curl err=%d, '%s')\n", curlErrCode, curl_easy_strerror(curlErrCode));
}

/// <summary>
///     Download a web page over HTTPS protocol using cURL.
/// </summary>
static void PerformWebhookPOST(float temp)
{
    CURL *curlHandle = NULL;
    CURLcode res = 0;
    MemoryBlock block = {.data = NULL, .size = 0};
    char *certificatePath = NULL;

    bool isNetworkingReady = false;
    if ((Networking_IsNetworkingReady(&isNetworkingReady) < 0) || !isNetworkingReady) {
        Log_Debug("\nNot doing download because network is not up.\n");
        goto exitLabel;
    }

    Log_Debug("\n -===- Starting POST -===-\n");
		
    // Init the cURL library.
    if ((res = curl_global_init(CURL_GLOBAL_ALL)) != CURLE_OK) {
        LogCurlError("curl_global_init", res);
        goto exitLabel;
    }

    if ((curlHandle = curl_easy_init()) == NULL) {
        Log_Debug("curl_easy_init() failed\n");
        goto cleanupLabel;
    }

    // Specify URL to POST to.
    // Important: any change in the domain name must be reflected in the AllowedConnections
    // capability in app_manifest.json.
    if ((res = curl_easy_setopt(curlHandle, CURLOPT_URL, "YOUR_FLOW_OR_CONNECTOR_OR_LOGIC_APP_URL_HERE")) != CURLE_OK) {
        LogCurlError("curl_easy_setopt CURLOPT_URL", res);
		    goto cleanupLabel;
    }

    // Set output level to verbose.
    if ((res = curl_easy_setopt(curlHandle, CURLOPT_VERBOSE, 1L)) != CURLE_OK) {
        LogCurlError("curl_easy_setopt CURLOPT_VERBOSE", res);
        goto cleanupLabel;
    }

    // Get the full path to the certificate file used to authenticate the HTTPS server identity.
    // The DigiCertGlobalRootCA.pem file is the certificate that is used to verify the
    // server identity.
    certificatePath = Storage_GetAbsolutePathInImagePackage("certs/DigiCertGlobalRootCA.pem");
    if (certificatePath == NULL) {
        Log_Debug("The certificate path could not be resolved: errno=%d (%s)\n", errno,
                  strerror(errno));
        goto cleanupLabel;
    }

    // Set the path for the certificate file that cURL uses to validate the server certificate.
    if ((res = curl_easy_setopt(curlHandle, CURLOPT_CAINFO, certificatePath)) != CURLE_OK) {
        LogCurlError("curl_easy_setopt CURLOPT_CAINFO", res);
        goto cleanupLabel;
    }
	    
	if ((res = curl_easy_setopt(curlHandle, CURLOPT_POST, 1L)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_POST", res);
		goto cleanupLabel;
	}

	char str[150];
	char tmp[6];
		
	snprintf(tmp, sizeof(tmp), "%.2f", temp);
	strcpy(str, "{\"Temp\":");
	strcat(str, tmp);
	strcat(str, "}" );
	
	
	if ((res = curl_easy_setopt(curlHandle, CURLOPT_POSTFIELDS, str)) != CURLE_OK) {
		LogCurlError("curl_easy_setopt CURLOPT_POSTFIELDS", res);
		goto cleanupLabel;
	}
	
	struct curl_slist* hs = NULL;
	hs = curl_slist_append(hs, "Content-Type: application/json");
	curl_easy_setopt(curlHandle, CURLOPT_HTTPHEADER, hs);
	
    // Perform the POST action
    if ((res = curl_easy_perform(curlHandle)) != CURLE_OK) {
        LogCurlError("curl_easy_perform", res);
    } else {
        Log_Debug("\n -===- Downloaded content (%zu bytes): -===-\n", block.size);
        Log_Debug("%s\n", block.data);
    }
	
cleanupLabel:
    // Clean up allocated memory.
    free(block.data);
    free(certificatePath);
    // Clean up sample's cURL resources.
    curl_easy_cleanup(curlHandle);
    // Clean up cURL library's resources.
    curl_global_cleanup();
    Log_Debug("\n -===- End of download -===-\n");

exitLabel:
    return;
}

/// <summary>
///     The timer event handler.
/// </summary>
static void TimerEventHandler(EventData *eventData)
{
    if (ConsumeTimerFdEvent(webpageDownloadTimerFd) != 0) {
        terminationRequired = true;
        return;
    }

float temp = GetTemp();
  PerformWebhookPOST(temp);

}

// event handler data structures. Only the event handler field needs to be populated.
static EventData timerEventData = {.eventHandler = &TimerEventHandler};

/// <summary>
///     Set up SIGTERM termination handler and event handlers.
/// </summary>
/// <returns>0 on success, or -1 on failure</returns>
static int InitPeripheralsAndHandlers(void)
{
    struct sigaction action;
    memset(&action, 0, sizeof(struct sigaction));
    action.sa_handler = TerminationHandler;
    sigaction(SIGTERM, &action, NULL);

    epollFd = CreateEpollFd();
    if (epollFd < 0) {
        return -1;
    }

	if (initI2c() == -1) {
		return -1;
	}

    // Issue an HTTPS request at the specified period.
    struct timespec sixtySeconds = {60, 0};
    webpageDownloadTimerFd =
        CreateTimerFdAndAddToEpoll(epollFd, &sixtySeconds, &timerEventData, EPOLLIN);
    if (webpageDownloadTimerFd < 0) {
        return -1;
    }

    return 0;
}

/// <summary>
///     Clean up the resources previously allocated.
/// </summary>
static void ClosePeripheralsAndHandlers(void)
{
    // Close the timer and epoll file descriptors.
    CloseFdAndPrintError(webpageDownloadTimerFd, "WebpageDownloadTimer");
    CloseFdAndPrintError(epollFd, "Epoll");
}

/// <summary>
///     Main entry point for this sample.
/// </summary>
int main(int argc, char *argv[])
{
    Log_Debug("Temperature to Flow Sample. Tom Morgan. thoughtstuff.co.uk\n");
    Log_Debug("This sample periodically posts the temperature to a Flow endpoint.");

    if (InitPeripheralsAndHandlers() != 0) {
        terminationRequired = true;
    } else {
        // Do the webhook post immediately.
		float temp = GetTemp();
		PerformWebhookPOST(temp);

    }

    // Use epoll to wait for events and trigger handlers, until an error or SIGTERM happens
    while (!terminationRequired) {
        if (WaitForEventAndCallHandler(epollFd) != 0) {
            terminationRequired = true;
        }
    }

	ClosePeripheralsAndHandlers();
    Log_Debug("Application exiting.\n");
    return 0;
}
